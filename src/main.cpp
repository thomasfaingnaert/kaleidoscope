#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "llvm/IR/Constants.h"
#include "llvm/IR/IRBuilder.h"
#include "llvm/IR/Instructions.h"
#include "llvm/IR/LLVMContext.h"
#include "llvm/IR/LegacyPassManager.h"
#include "llvm/IR/Module.h"
#include "llvm/IR/Value.h"
#include "llvm/IR/Verifier.h"
#include "llvm/Support/TargetSelect.h"
#include "llvm/Transforms/InstCombine/InstCombine.h"
#include "llvm/Transforms/Scalar.h"
#include "llvm/Transforms/Scalar/GVN.h"

#include "KaleidoscopeJIT.h"

/* Lexer */

// We reserve [0, 255] for tokens corresponding to one ASCII character
enum Token {
  tok_eof = -1,

  // commands
  tok_def = -2,
  tok_extern = -3,

  // primary
  tok_identifier = -4,
  tok_number = -5,

  // control flow
  tok_if = -6,
  tok_then = -7,
  tok_else = -8,
  tok_for = -9,
  tok_in = -10,
};

static std::string IdentifierStr; // Filled in if tok_identifier
static double NumVal;             // Filled in if tok_number

// Returns the next token from standard input
static int gettok() {
  static int LastChar = ' '; // Last character read, but not processed

  // Skip any whitespace
  while (isspace(LastChar))
    LastChar = getchar();

  // Identifiers and keywords
  if (isalpha(LastChar)) { // [a-zA-Z][a-zA-Z0-9]*
    IdentifierStr = LastChar;

    while (isalnum(LastChar = getchar()))
      IdentifierStr += LastChar;

    // Check for keywords
    if (IdentifierStr == "def")
      return tok_def;
    if (IdentifierStr == "extern")
      return tok_extern;
    if (IdentifierStr == "if")
      return tok_if;
    if (IdentifierStr == "then")
      return tok_then;
    if (IdentifierStr == "else")
      return tok_else;
    if (IdentifierStr == "for")
      return tok_for;
    if (IdentifierStr == "in")
      return tok_in;

    // Not a keyword, so token is an identifier
    return tok_identifier;
  }

  // Numeric literals
  // FIXME: This will incorrectly lex e.g. "1.2.3" as a number
  if (isdigit(LastChar) || LastChar == '.') { // [0-9.]+
    std::string NumStr;

    do {
      NumStr += LastChar;
      LastChar = getchar();
    } while (isdigit(LastChar) || LastChar == '.');

    NumVal = strtod(NumStr.c_str(), 0);
    return tok_number;
  }

  // Ignore comments
  if (LastChar == '#') { // Start with '#' until end of line
    do {
      LastChar = getchar();
    } while (LastChar != EOF && LastChar != '\n' && LastChar != '\r');

    // Get the next token (unless we're at EOF)
    if (LastChar != EOF)
      return gettok();
  }

  // Check for EOF. Don't eat it!
  if (LastChar == EOF)
    return tok_eof;

  // Otherwise, just return the character as its ASCII value
  int CurChar = LastChar;
  LastChar = getchar();
  return CurChar;
}

// The current token the parser is looking at.
static int CurTok;

// Read another token from the lexer and update CurTok.
static int getNextToken() { return CurTok = gettok(); }

/* AST */

// Base class for all expressions
class ExprAST {
public:
  virtual ~ExprAST() {}
  virtual llvm::Value *codegen() = 0;
};

// Numeric literals, such as "1.0"
class NumberExprAST : public ExprAST {
  double Val;

public:
  NumberExprAST(double Val) : Val(Val) {}
  virtual llvm::Value *codegen();
};

// Variable expression, i.e. a reference to a variable
class VariableExprAST : public ExprAST {
  std::string Name;

public:
  VariableExprAST(const std::string &Name) : Name(Name) {}
  virtual llvm::Value *codegen();
};

// If-then-else expression
class IfExprAST : public ExprAST {
  std::unique_ptr<ExprAST> Cond, Then, Else;

public:
  IfExprAST(std::unique_ptr<ExprAST> Cond, std::unique_ptr<ExprAST> Then,
            std::unique_ptr<ExprAST> Else)
      : Cond(std::move(Cond)), Then(std::move(Then)), Else(std::move(Else)) {}

  virtual llvm::Value *codegen();
};

// For expression
class ForExprAST : public ExprAST {
  std::string VarName;
  std::unique_ptr<ExprAST> Start, Cond, Step, Body;

public:
  ForExprAST(const std::string &VarName, std::unique_ptr<ExprAST> Start,
             std::unique_ptr<ExprAST> Cond, std::unique_ptr<ExprAST> Step,
             std::unique_ptr<ExprAST> Body)
      : VarName(VarName), Start(std::move(Start)), Cond(std::move(Cond)),
        Step(std::move(Step)), Body(std::move(Body)) {}

  virtual llvm::Value *codegen();
};

// Binary expression
class BinaryExprAST : public ExprAST {
  char Op;
  std::unique_ptr<ExprAST> LHS, RHS;

public:
  BinaryExprAST(char op, std::unique_ptr<ExprAST> LHS,
                std::unique_ptr<ExprAST> RHS)
      : Op(op), LHS(std::move(LHS)), RHS(std::move(RHS)) {}
  virtual llvm::Value *codegen();
};

// Function call
class CallExprAST : public ExprAST {
  std::string Callee;
  std::vector<std::unique_ptr<ExprAST>> Args;

public:
  CallExprAST(const std::string &Callee,
              std::vector<std::unique_ptr<ExprAST>> Args)
      : Callee(Callee), Args(std::move(Args)) {}
  virtual llvm::Value *codegen();
};

// Function prototype, i.e. the name of the function and of its arguments (and
// thus implicitly also the number of arguments of the function).
class PrototypeAST {
  std::string Name;
  std::vector<std::string> Args;

public:
  PrototypeAST(const std::string &Name, const std::vector<std::string> &Args)
      : Name(Name), Args(Args) {}

  std::string getName() const { return Name; }
  virtual llvm::Function *codegen();
};

// The function definition itself
class FunctionAST {
  std::unique_ptr<PrototypeAST> Proto;
  std::unique_ptr<ExprAST> Body;

public:
  FunctionAST(std::unique_ptr<PrototypeAST> Proto,
              std::unique_ptr<ExprAST> Body)
      : Proto(std::move(Proto)), Body(std::move(Body)) {}
  virtual llvm::Function *codegen();
};

/* Error handling */

std::unique_ptr<ExprAST> LogError(const char *Str) {
  fprintf(stderr, "LogError: %s\n", Str);
  return nullptr;
}

std::unique_ptr<PrototypeAST> LogErrorP(const char *Str) {
  LogError(Str);
  return nullptr;
}

llvm::Value *LogErrorV(const char *Str) {
  LogError(Str);
  return nullptr;
}

/* Parser */

std::unique_ptr<ExprAST> ParseExpression(); // forward decl

// ifexpr ::= 'if' expression 'then' expression 'else' expression
std::unique_ptr<ExprAST> ParseIfExpr() {
  getNextToken(); // Eat the 'if'

  // get the condition
  auto Cond = ParseExpression();
  if (!Cond)
    return nullptr;

  if (CurTok != tok_then)
    return LogError("expected 'then'");

  getNextToken(); // Eat the 'then'

  auto Then = ParseExpression();
  if (!Then)
    return nullptr;

  if (CurTok != tok_else)
    return LogError("expected 'else'");

  getNextToken(); // Eat the 'else'

  auto Else = ParseExpression();
  if (!Else)
    return nullptr;

  return std::make_unique<IfExprAST>(std::move(Cond), std::move(Then),
                                     std::move(Else));
}

// forexpr ::= 'for' identifier '=' expression ',' expression [',' expression]
//             'in' expression
std::unique_ptr<ExprAST> ParseForExpr() {
  getNextToken(); // Eat the 'for'

  if (CurTok != tok_identifier)
    return LogError("expected identifier after for");

  std::string IdName = IdentifierStr;
  getNextToken(); // Eat the identifier

  if (CurTok != '=')
    return LogError("expected '=' after for");
  getNextToken(); // Eat the '='

  auto Start = ParseExpression();
  if (!Start)
    return nullptr;

  if (CurTok != ',')
    return LogError("expected ',' after for start value");
  getNextToken(); // Eat the ','

  auto Cond = ParseExpression();
  if (!Cond)
    return nullptr;

  std::unique_ptr<ExprAST> Step;

  // Check if the optional step value is present
  if (CurTok == ',') {
    getNextToken(); // Eat the ','

    Step = ParseExpression();
    if (!Step)
      return nullptr;
  }

  if (CurTok != tok_in)
    return LogError("expected 'in' after for");
  getNextToken(); // Eat the 'in'

  auto Body = ParseExpression();
  if (!Body)
    return nullptr;

  return std::make_unique<ForExprAST>(IdName, std::move(Start), std::move(Cond),
                                      std::move(Step), std::move(Body));
}

// numberexpr ::= number
std::unique_ptr<ExprAST> ParseNumberExpr() {
  auto Result = std::make_unique<NumberExprAST>(NumVal);
  getNextToken(); // Consume the number
  return Result;
}

// parenexpr ::= '(' expression ')'
std::unique_ptr<ExprAST> ParseParenExpr() {
  getNextToken(); // Eat the '('

  auto V = ParseExpression();
  if (!V)
    return nullptr;

  if (CurTok != ')')
    return LogError("expected ')'");

  getNextToken(); // Eat the ')'

  return V;
}

// identifierexpr ::= identifier
//                ::= identifier '(' expression* ')'
std::unique_ptr<ExprAST> ParseIdentifierExpr() {
  std::string IdName = IdentifierStr;

  getNextToken(); // eat the identifier

  if (CurTok != '(') // simple variable reference
    return std::make_unique<VariableExprAST>(IdName);

  // Function call
  getNextToken(); // Eat the '('
  std::vector<std::unique_ptr<ExprAST>> Args;
  if (CurTok != ')') {
    while (true) {
      if (auto Arg = ParseExpression())
        Args.push_back(std::move(Arg));
      else
        return nullptr;

      if (CurTok == ')') // found end of argument list
        break;

      if (CurTok != ',')
        return LogError("Expected ')' or ',' in argument list");

      getNextToken(); // eat ','
    }
  }

  getNextToken(); // eat ')'
  return std::make_unique<CallExprAST>(IdName, std::move(Args));
}

// primary ::= identifierexpr
//         ::= numberexpr
//         ::= parenexpr
//         ::= ifexpr
//         ::= forexpr
std::unique_ptr<ExprAST> ParsePrimary() {
  switch (CurTok) {
  default:
    return LogError("unknown token when expecting an expression");
  case tok_identifier:
    return ParseIdentifierExpr();
  case tok_number:
    return ParseNumberExpr();
  case '(':
    return ParseParenExpr();
  case tok_if:
    return ParseIfExpr();
  case tok_for:
    return ParseForExpr();
  }
}

// Maps operators to precedence
std::map<char, int> BinopPrecedence;

int GetTokPrecedence() {
  if (!isascii(CurTok))
    return -1;

  // Make sure the token is a declared binop
  int TokPrec = BinopPrecedence[CurTok];
  if (TokPrec <= 0)
    return -1;
  return TokPrec;
}

// binoprhs ::= (op primary)*
std::unique_ptr<ExprAST> ParseBinOpRHS(int MinPrecedence,
                                       std::unique_ptr<ExprAST> LHS) {
  // Parse all (op, primary) pairs
  while (true) {
    // Get the current token's precedence
    int CurPrecedence = GetTokPrecedence();

    // Only consume operators with precedence higher than the minimum we are
    // allowed to eat. Note that this case also handles non operators, which
    // have precedence -1.
    if (CurPrecedence < MinPrecedence) {
      return LHS;
    }

    // In the other case, we know that the token is a binary operator.
    int BinOp = CurTok;
    getNextToken(); // eat binop

    // Parse the primary expression after the binop
    auto RHS = ParsePrimary();
    if (!RHS)
      return nullptr;

    // If the next token's precedence is higher, then we let that operator take
    // RHS as its LHS.
    int NextPrecedence = GetTokPrecedence();
    if (NextPrecedence > CurPrecedence) {
      // Parse the whole right hand side
      RHS = ParseBinOpRHS(CurPrecedence + 1, std::move(RHS));
    }

    // Merge LHS and RHS
    LHS =
        std::make_unique<BinaryExprAST>(BinOp, std::move(LHS), std::move(RHS));
  }
}

// expression ::= primary binoprhs
std::unique_ptr<ExprAST> ParseExpression() {
  auto LHS = ParsePrimary();
  if (!LHS)
    return nullptr;

  return ParseBinOpRHS(0, std::move(LHS));
}

// prototype ::= identifier '(' identifier* ')'
std::unique_ptr<PrototypeAST> ParsePrototype() {
  if (CurTok != tok_identifier)
    return LogErrorP("Expected function name in prototype");

  std::string FunctionName = IdentifierStr;
  getNextToken(); // eat identifier

  if (CurTok != '(')
    return LogErrorP("Expected '(' in prototype");

  // Read the arguments
  std::vector<std::string> ArgNames;
  while (getNextToken() == tok_identifier)
    ArgNames.push_back(IdentifierStr);
  if (CurTok != ')')
    return LogErrorP("Expected ')' in prototype");

  getNextToken(); // eat ')'

  return std::make_unique<PrototypeAST>(FunctionName, ArgNames);
}

// definition ::= 'def' prototype expression
std::unique_ptr<FunctionAST> ParseDefinition() {
  getNextToken(); // eat 'def'
  auto Proto = ParsePrototype();
  if (!Proto)
    return nullptr;

  if (auto E = ParseExpression())
    return std::make_unique<FunctionAST>(std::move(Proto), std::move(E));
  return nullptr;
}

// external ::= 'extern' prototype
std::unique_ptr<PrototypeAST> ParseExtern() {
  getNextToken(); // eat 'extern'
  return ParsePrototype();
}

// Allow the user to type top-level expressions (e.g. for use in the REPL).
// Do this by wrapping them in zero-argument functions.
// toplevelexpr ::= expression
std::unique_ptr<FunctionAST> ParseTopLevelExpr() {
  if (auto E = ParseExpression()) {
    // Make an anonymous prototype
    auto Proto = std::make_unique<PrototypeAST>("__anon_expr",
                                                std::vector<std::string>());
    return std::make_unique<FunctionAST>(std::move(Proto), std::move(E));
  }
  return nullptr;
}

/* Global values used for codegen */
llvm::LLVMContext TheContext;
llvm::IRBuilder<> Builder(TheContext);
std::unique_ptr<llvm::Module> TheModule;
std::unique_ptr<llvm::legacy::FunctionPassManager> TheFPM;
std::unique_ptr<llvm::orc::KaleidoscopeJIT> TheJIT;
std::map<llvm::StringRef, llvm::Value *> NamedValues; // symbol table
std::map<std::string, std::unique_ptr<PrototypeAST>> FunctionProtos;

/* Codegen */
llvm::Function *getFunction(const std::string &name) {
  // First, check if the function is already present in the module
  if (auto F = TheModule->getFunction(name))
    return F;

  // If not, check if we can codegen the declaration from some existing
  // prototype
  auto FI = FunctionProtos.find(name);
  if (FI != FunctionProtos.end())
    return FI->second->codegen();

  // No existing prototype exists, so return null
  return nullptr;
}

llvm::Value *NumberExprAST::codegen() {
  return llvm::ConstantFP::get(TheContext, llvm::APFloat(Val));
}

llvm::Value *VariableExprAST::codegen() {
  // Look this variable up in the symbol table
  llvm::Value *V = NamedValues[Name];
  if (!V)
    LogErrorV("Unknown variable name");
  return V;
}

llvm::Value *IfExprAST::codegen() {
  llvm::Value *CondV = Cond->codegen();
  if (!CondV)
    return nullptr;

  // Like all values, condition is floating point, so convert it to a bool by
  // using a != 0.0 comparison
  CondV = Builder.CreateFCmpONE(
      CondV, llvm::ConstantFP::get(TheContext, llvm::APFloat(0.0)), "ifcond");

  auto TheFunction = Builder.GetInsertBlock()->getParent();

  // Create basic blocks for the then and else cases, and the basic block that
  // merges them.
  llvm::BasicBlock *ThenBB =
      llvm::BasicBlock::Create(TheContext, "then", TheFunction);
  llvm::BasicBlock *ElseBB =
      llvm::BasicBlock::Create(TheContext, "else", TheFunction);
  llvm::BasicBlock *ContBB =
      llvm::BasicBlock::Create(TheContext, "ifcont", TheFunction);

  Builder.CreateCondBr(CondV, ThenBB, ElseBB);

  // Emit the 'then' expression
  Builder.SetInsertPoint(ThenBB);

  llvm::Value *ThenV = Then->codegen();
  if (!ThenV)
    return nullptr;

  Builder.CreateBr(ContBB);

  // NOTE: emitting the 'Then' expression may have changed the current basic
  // block (e.g. for nested if-expressions), so we need to explicitly set ThenBB
  // again for the PHI node to be correct.
  ThenBB = Builder.GetInsertBlock();

  // Emit the 'else' expression
  Builder.SetInsertPoint(ElseBB);

  llvm::Value *ElseV = Else->codegen();
  if (!ElseV)
    return nullptr;

  Builder.CreateBr(ContBB);

  // Same thing for the else
  ElseBB = Builder.GetInsertBlock();

  // Emit merge basic block
  Builder.SetInsertPoint(ContBB);
  auto PhiNode =
      Builder.CreatePHI(llvm::Type::getDoubleTy(TheContext), 2, "iftmp");

  PhiNode->addIncoming(ThenV, ThenBB);
  PhiNode->addIncoming(ElseV, ElseBB);

  return PhiNode;
}

llvm::Value *ForExprAST::codegen() {
  // Emit the start value first.
  llvm::Value *StartVal = Start->codegen();
  if (!StartVal)
    return nullptr;

  llvm::Function *TheFunction = Builder.GetInsertBlock()->getParent();
  llvm::BasicBlock *PreloopBB = Builder.GetInsertBlock();

  // Make a basic block for the loop.
  llvm::BasicBlock *LoopBB =
      llvm::BasicBlock::Create(TheContext, "loop", TheFunction);

  // Create an explicit fallthrough from the preloop BB to the loop BB.
  Builder.CreateBr(LoopBB);

  // Start inserting in the loop BB.
  Builder.SetInsertPoint(LoopBB);

  // Create a PHI node for the loop variable.
  llvm::PHINode *PHINode =
      Builder.CreatePHI(llvm::Type::getDoubleTy(TheContext), 2, VarName);
  PHINode->addIncoming(StartVal, PreloopBB);

  // Update the symbol table in the loop, so that 'VarName' refers to this PHI
  // node. Save the old value, so we can restore it later.
  llvm::Value *OldValue = NamedValues[VarName];
  NamedValues[VarName] = PHINode;

  // Emit the body of the loop. This, like any other expression, can change the
  // current BB. Note that we don't care about the return value (i.e. the value
  // computed by the body).
  if (!Body->codegen())
    return nullptr;

  // Emit the step value.
  llvm::Value *StepValue = nullptr;
  if (Step) { // Step is specified.
    StepValue = Step->codegen();
    if (!StepValue)
      return nullptr;
  } else { // No step specified, so use 1.0.
    StepValue = llvm::ConstantFP::get(TheContext, llvm::APFloat(1.0));
  }

  // Get the next value of the loop variable.
  llvm::Value *NextValue = Builder.CreateFAdd(PHINode, StepValue, "nextvar");

  // Evaluate the loop condition.
  llvm::Value *CondValue = Cond->codegen();
  if (!CondValue)
    return nullptr;

  // Like all values, the condition is floating point, so convert it to a bool
  // by using a != 0.0 comparison.
  CondValue = Builder.CreateFCmpONE(
      CondValue, llvm::ConstantFP::get(TheContext, llvm::APFloat(0.0)),
      "loopcond");

  //  Create a BB for after the loop.
  llvm::BasicBlock *AfterBB =
      llvm::BasicBlock::Create(TheContext, "afterloop", TheFunction);

  // Create the loop branch.
  Builder.CreateCondBr(CondValue, LoopBB, AfterBB);

  // Remember the current basic block for the PHI-node.
  llvm::BasicBlock *LoopEndBB = Builder.GetInsertBlock();

  // From this point on, start inserting in the end block.
  Builder.SetInsertPoint(AfterBB);

  // Add the other value of the PHI node.
  PHINode->addIncoming(NextValue, LoopEndBB);

  // Restore the symbol-table mapping.
  if (OldValue)
    NamedValues[VarName] = OldValue;
  else
    NamedValues.erase(VarName);

  // A for expression always returns 0.
  return llvm::Constant::getNullValue(llvm::Type::getDoubleTy(TheContext));
}

llvm::Value *BinaryExprAST::codegen() {
  llvm::Value *L = LHS->codegen();
  llvm::Value *R = RHS->codegen();
  if (!L || !R)
    return nullptr;

  switch (Op) {
  case '+':
    return Builder.CreateFAdd(L, R, "addtmp");
  case '-':
    return Builder.CreateFSub(L, R, "subtmp");
  case '*':
    return Builder.CreateFMul(L, R, "multmp");
  case '<':
    L = Builder.CreateFCmpULT(L, R, "cmptmp");
    // fcmp returns i1. Convert it to a double.
    return Builder.CreateUIToFP(L, llvm::Type::getDoubleTy(TheContext),
                                "booltmp");
  default:
    return LogErrorV("invalid binary operator");
  }
}

llvm::Value *CallExprAST::codegen() {
  // Look up the name of the function
  llvm::Function *CalleeF = getFunction(Callee);
  if (!CalleeF)
    return LogErrorV("Unknown function referenced");

  // Check if the arguments match. Of course, because all variables are the same
  // type, we only need to check the number of arguments.
  if (CalleeF->arg_size() != Args.size())
    return LogErrorV("Incorrect number of arguments passed");

  std::vector<llvm::Value *> ArgsV;
  for (auto &Arg : Args) {
    llvm::Value *EmittedArg = Arg->codegen();
    if (!EmittedArg)
      return nullptr;
    ArgsV.push_back(EmittedArg);
  }

  return Builder.CreateCall(CalleeF, ArgsV, "calltmp");
}

llvm::Function *PrototypeAST::codegen() {
  // Create the function's type
  std::vector<llvm::Type *> Doubles(Args.size(),
                                    llvm::Type::getDoubleTy(TheContext));
  llvm::FunctionType *FT = llvm::FunctionType::get(
      llvm::Type::getDoubleTy(TheContext), Doubles, false);

  llvm::Function *F = llvm::Function::Create(
      FT, llvm::Function::ExternalLinkage, Name, TheModule.get());

  // Set the names of the arguments
  unsigned i = 0;
  for (auto &Arg : F->args()) {
    Arg.setName(Args[i++]);
  }

  return F;
}

llvm::Function *FunctionAST::codegen() {
  // Transfer ownership of the prototype to the FunctionProtos map, but keep a
  // reference so we can still use it.
  auto &P = *Proto;
  FunctionProtos[P.getName()] = std::move(Proto);

  // First check for an existing function in the module (e.g. from an extern
  // declaration)
  llvm::Function *TheFunction = getFunction(P.getName());

  if (!TheFunction)
    return nullptr;

  if (!TheFunction->empty())
    return static_cast<llvm::Function *>(
        LogErrorV("Function cannot be redefined"));

  // Create a basic block to insert instructions into
  llvm::BasicBlock *BB =
      llvm::BasicBlock::Create(TheContext, "entry", TheFunction);
  Builder.SetInsertPoint(BB);

  // Record the function arguments in the NamedValues map.
  NamedValues.clear(); // clear arguments of previous function
  for (auto &Arg : TheFunction->args())
    NamedValues[Arg.getName()] = &Arg;

  if (llvm::Value *RetVal = Body->codegen()) {
    // Finish the function
    Builder.CreateRet(RetVal);

    // Validate the generated code, checking for consistency
    llvm::verifyFunction(*TheFunction, &llvm::errs());

    // Optimize the function
    TheFPM->run(*TheFunction);

    return TheFunction;
  }

  // Error reading body, remove the function from the module.
  TheFunction->eraseFromParent();
  return nullptr;
}

/* Top level parsing */
void InitializeModuleAndPassManager();

void HandleDefinition() {
  if (auto FnAST = ParseDefinition()) {
    if (auto FnIR = FnAST->codegen()) {
      std::cerr << "Read function definition:\n";
      FnIR->print(llvm::errs());
      std::cerr << "\n";

      TheJIT->addModule(std::move(TheModule));
      InitializeModuleAndPassManager();
    }
  } else {
    getNextToken(); // simple error recovery: skip to next token
  }
}

void HandleExtern() {
  if (auto ProtoAST = ParseExtern()) {
    if (auto ProtoIR = ProtoAST->codegen()) {
      std::cerr << "Read extern:\n";
      ProtoIR->print(llvm::errs());
      std::cerr << "\n";

      FunctionProtos[ProtoAST->getName()] = std::move(ProtoAST);
    }
  } else {
    getNextToken(); // simple error recovery: skip to next token
  }
}

void HandleTopLevelExpression() {
  // Evaluate a top-level expression into an anonymous function
  if (auto ExprAST = ParseTopLevelExpr()) {
    if (auto ExprIR = ExprAST->codegen()) {
      // Print LLVM IR
      std::cerr << "Read a top-level expression:\n";
      ExprIR->print(llvm::errs());
      std::cerr << "\n";

      // JIT the module containing the expression
      auto Handle = TheJIT->addModule(std::move(TheModule));

      // Adding the module to the JIT sets TheModule to nullptr, so we need to
      // create a new module and pass manager.
      InitializeModuleAndPassManager();

      // Search the JIT for the __anon_expr symbol
      auto AnonSymbol = TheJIT->findSymbol("__anon_expr");
      assert(AnonSymbol && "Function __anon_expr not found");

      // Cast the symbol's address to the right type, i.e. a pointer to a
      // function taking no arguments and returning a double. This way, we can
      // call it as a native function.
      typedef double(anon_expr_func)();
      anon_expr_func *FuncPtr = reinterpret_cast<anon_expr_func *>(
          llvm::cantFail(AnonSymbol.getAddress()));
      std::cerr << "= " << FuncPtr() << std::endl;

      // Delete the anonymous expression module from the JIT.
      TheJIT->removeModule(Handle);
    }
  } else {
    getNextToken(); // simple error recovery: skip to next token
  }
}

void MainLoop() {
  while (true) {
    switch (CurTok) {
    case tok_eof:
      return; // end of file, stop running
    case ';':
      getNextToken(); // ignore top level ';'
      break;
    case tok_def:
      HandleDefinition();
      std::cerr << "\nready> ";
      break;
    case tok_extern:
      HandleExtern();
      std::cerr << "\nready> ";
      break;
    default:
      HandleTopLevelExpression();
      std::cerr << "\nready> ";
      break;
    }
  }
}

void InitializeModuleAndPassManager() {
  // Initialise the module
  TheModule = std::make_unique<llvm::Module>("my cool jit", TheContext);

  // Set the data layout for the module
  TheModule->setDataLayout(TheJIT->getTargetMachine().createDataLayout());

  // Create a new pass manager attached to it
  TheFPM = std::make_unique<llvm::legacy::FunctionPassManager>(TheModule.get());

  // Do simple "peephole" optimizations and bit-twiddling optimizations
  TheFPM->add(llvm::createInstructionCombiningPass());

  // Reassociate expressions
  TheFPM->add(llvm::createReassociatePass());

  // Eliminate common subexpressions
  TheFPM->add(llvm::createGVNPass());

  // Simplify the CFG (deleting unreachable blocks, ...)
  TheFPM->add(llvm::createCFGSimplificationPass());

  // Run the initialization for all passes
  TheFPM->doInitialization();
}

/* Library routines that can be 'extern'ed from user code */

#ifdef _WIN32
#define DLLEXPORT __declspec(dllexport)
#else
#define DLLEXPORT /* blank */
#endif

extern "C" DLLEXPORT double putchard(double x) {
  std::cerr << static_cast<char>(x);
  return 0;
}

extern "C" DLLEXPORT double printd(double x) {
  std::cerr << x << '\n';
  return 0;
}

/* Main routine */

int main(int argc, char *argv[]) {
  // Initialization of native target
  llvm::InitializeNativeTarget();
  llvm::InitializeNativeTargetAsmPrinter();
  llvm::InitializeNativeTargetAsmParser();

  // Initialize the JIT
  TheJIT = std::make_unique<llvm::orc::KaleidoscopeJIT>();

  // Register default binary operators.
  BinopPrecedence['<'] = 10;
  BinopPrecedence['+'] = 20;
  BinopPrecedence['-'] = 20;
  BinopPrecedence['*'] = 40;

  // Initialization
  InitializeModuleAndPassManager();

  // Prime the first token
  std::cerr << "ready> ";
  getNextToken();

  // Run main loop
  MainLoop();

  return 0;
}
