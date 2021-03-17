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
#include "llvm/Transforms/Utils.h"

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

  // operators
  tok_binary = -11,
  tok_unary = -12,

  // var definition,
  tok_var = -13,
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
    if (IdentifierStr == "binary")
      return tok_binary;
    if (IdentifierStr == "unary")
      return tok_unary;
    if (IdentifierStr == "var")
      return tok_var;

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
  std::string getName() const { return Name; }
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

// Expression class for var/in
class VarExprAST : public ExprAST {
  std::vector<std::pair<std::string, std::unique_ptr<ExprAST>>> VarNames;
  std::unique_ptr<ExprAST> Body;

public:
  VarExprAST(
      std::vector<std::pair<std::string, std::unique_ptr<ExprAST>>> VarNames,
      std::unique_ptr<ExprAST> Body)
      : VarNames(std::move(VarNames)), Body(std::move(Body)) {}

  virtual llvm::Value *codegen();
};

// Unary expression
class UnaryExprAST : public ExprAST {
  char Op;
  std::unique_ptr<ExprAST> Operand;

public:
  UnaryExprAST(char Op, std::unique_ptr<ExprAST> Operand)
      : Op(Op), Operand(std::move(Operand)) {}
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
// It is also used for operator definitions.
class PrototypeAST {
  std::string Name;
  std::vector<std::string> Args;
  bool IsOperator;
  unsigned Precedence; // Precedence if a binary op.

public:
  PrototypeAST(const std::string &Name, const std::vector<std::string> &Args,
               bool IsOperator = false, unsigned Prec = 0)
      : Name(Name), Args(Args), IsOperator(IsOperator), Precedence(Prec) {}

  std::string getName() const { return Name; }
  virtual llvm::Function *codegen();

  bool isUnaryOp() const { return IsOperator && Args.size() == 1; }
  bool isBinaryOp() const { return IsOperator && Args.size() == 2; }

  char getOperatorName() const {
    assert(isUnaryOp() || isBinaryOp());
    // Name of the function is 'unaryX' or 'binaryX', where X is the operator.
    return Name[Name.size() - 1];
  }

  unsigned getPrecedence() const { return Precedence; }
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
std::unique_ptr<ExprAST> ParseUnary();      // forward decl

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

// varexpr ::= 'var' identifier ('=' expression)?
//             (',' identifier ('=' expression)?)* 'in' expression
std::unique_ptr<ExprAST> ParseVarExpr() {
  getNextToken(); // eat the var.

  std::vector<std::pair<std::string, std::unique_ptr<ExprAST>>> VarNames;

  // At least one variable name is required.
  if (CurTok != tok_identifier)
    return LogError("expected identifier after var");

  while (true) {
    std::string Name = IdentifierStr;
    getNextToken(); // eat identifier.

    // Read the optional initializer.
    std::unique_ptr<ExprAST> Init;
    if (CurTok == '=') {
      getNextToken(); // eat the '='.

      Init = ParseExpression();
      if (!Init)
        return nullptr;
    }

    VarNames.push_back(std::make_pair(Name, std::move(Init)));

    // End of var list, exit loop.
    if (CurTok != ',')
      break;
    getNextToken(); // eat the ','.

    if (CurTok != tok_identifier)
      return LogError("expected identifier list after var");
  }

  // At this point, we have to have 'in'.
  if (CurTok != tok_in)
    return LogError("expected 'in' keyword after 'var'");
  getNextToken(); // eat 'in'.

  auto Body = ParseExpression();
  if (!Body)
    return nullptr;

  return std::make_unique<VarExprAST>(std::move(VarNames), std::move(Body));
}

// primary ::= identifierexpr
//         ::= numberexpr
//         ::= parenexpr
//         ::= ifexpr
//         ::= forexpr
//         ::= varexpr
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
  case tok_var:
    return ParseVarExpr();
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

// binoprhs ::= (op unary)*
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

    // Parse the unary expression after the binop
    auto RHS = ParseUnary();
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

// unary ::= primary
//       ::= CHAR unary
std::unique_ptr<ExprAST> ParseUnary() {
  // If the current token is not an operator, it must be a primary expr.
  if (!isascii(CurTok) || CurTok == '(' || CurTok == ',')
    return ParsePrimary();

  // If this is a unary operator, read it.
  int Opc = CurTok;
  getNextToken();
  if (auto Operand = ParseUnary())
    return std::make_unique<UnaryExprAST>(Opc, std::move(Operand));

  return nullptr;
}

// expression ::= unary binoprhs
std::unique_ptr<ExprAST> ParseExpression() {
  auto LHS = ParseUnary();
  if (!LHS)
    return nullptr;

  return ParseBinOpRHS(0, std::move(LHS));
}

// prototype ::= identifier '(' identifier* ')'
//           ::= binary CHAR number? '(' identifier identifier ')'
//           ::= unary CHAR '(' identifier ')'
std::unique_ptr<PrototypeAST> ParsePrototype() {
  std::string FunctionName;
  unsigned Kind = 0; // 0 = normal function, 1 = unary op, 2 = binary op
  unsigned BinaryPrecedence = 30;

  switch (CurTok) {
  default:
    return LogErrorP("Expected function name in prototype");
    break;

  case tok_identifier:
    // regular function prototype
    Kind = 0;
    FunctionName = IdentifierStr;
    getNextToken(); // eat identifier
    break;

  case tok_unary:
    // unary operator
    Kind = 1;
    getNextToken(); // eat 'unary' keyword

    if (!isascii(CurTok))
      return LogErrorP("Expected unary operator");
    FunctionName = "unary";
    FunctionName += static_cast<char>(CurTok);
    getNextToken(); // eat operator name
    break;

  case tok_binary:
    // binary operator
    Kind = 2;
    getNextToken(); // eat 'binary' keyword

    if (!isascii(CurTok))
      return LogErrorP("Expected binary operator");
    FunctionName = "binary";
    FunctionName += static_cast<char>(CurTok);
    getNextToken(); // eat operator name

    // Read precedence, if present
    if (CurTok == tok_number) {
      if (NumVal < 1 || NumVal > 100) {
        return LogErrorP("Invalid precedence: must be between 1 and 100");
      }
      BinaryPrecedence = static_cast<unsigned>(NumVal);
      getNextToken(); // eat precedence
    }

    break;
  }

  if (CurTok != '(')
    return LogErrorP("Expected '(' in prototype");

  // Read the arguments
  std::vector<std::string> ArgNames;
  while (getNextToken() == tok_identifier)
    ArgNames.push_back(IdentifierStr);
  if (CurTok != ')')
    return LogErrorP("Expected ')' in prototype");

  getNextToken(); // eat ')'

  // Verify the right number of arguments for operator
  if ((Kind != 0) && (ArgNames.size() != Kind)) {
    return LogErrorP("Invalid number of operands for operator");
  }

  return std::make_unique<PrototypeAST>(FunctionName, ArgNames, Kind != 0,
                                        BinaryPrecedence);
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
std::map<std::string, llvm::AllocaInst *> NamedValues; // symbol table
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

// CreateEntryBlockAlloca - Create an alloca instruction in the entry block of
// the function. This is used for mutable variables etc.
llvm::AllocaInst *CreateEntryBlockAlloca(llvm::Function *TheFunction,
                                         const std::string &VarName) {
  llvm::IRBuilder<> TheBuilder(&TheFunction->getEntryBlock(),
                               TheFunction->getEntryBlock().begin());

  return TheBuilder.CreateAlloca(llvm::Type::getDoubleTy(TheContext), 0,
                                 VarName);
}

llvm::Value *NumberExprAST::codegen() {
  return llvm::ConstantFP::get(TheContext, llvm::APFloat(Val));
}

llvm::Value *VariableExprAST::codegen() {
  // Look this variable up in the symbol table
  llvm::Value *V = NamedValues[Name];
  if (!V)
    return LogErrorV("Unknown variable name");

  // Load the value
  return Builder.CreateLoad(V, Name);
}

llvm::Value *VarExprAST::codegen() {
  std::vector<llvm::AllocaInst *> OldBindings;

  llvm::Function *TheFunction = Builder.GetInsertBlock()->getParent();

  // Register all variables and emit their initializer.
  for (unsigned i = 0; i != VarNames.size(); ++i) {
    std::string VarName = VarNames[i].first;
    ExprAST *Init = VarNames[i].second.get();

    // Emit the initializer before adding the variable to scope, this prevents
    // the initializer from referencing the variable itself. It also permits
    // things like: var a = 1 in
    //  var a = a in ... # refers to the outer 'a'.
    llvm::Value *InitVal;
    if (Init) {
      InitVal = Init->codegen();
      if (!InitVal)
        return nullptr;
    } else { // If initial value is not specified, use 0.0.
      InitVal = llvm::ConstantFP::get(TheContext, llvm::APFloat(0.0));
    }

    llvm::AllocaInst *Alloca = CreateEntryBlockAlloca(TheFunction, VarName);
    Builder.CreateStore(InitVal, Alloca);

    // Remember the old variable binding so that we can restore the binding when
    // we unrecurse.
    OldBindings.push_back(NamedValues[VarName]);

    // Remember this binding.
    NamedValues[VarName] = Alloca;
  }

  // Codegen the body, now that all vars are in scope.
  llvm::Value *BodyVal = Body->codegen();
  if (!BodyVal)
    return nullptr;

  // Pop all our variables from scope.
  for (unsigned i = 0; i != VarNames.size(); ++i)
    NamedValues[VarNames[i].first] = OldBindings[i];

  // Return the body computation.
  return BodyVal;
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
  llvm::Function *TheFunction = Builder.GetInsertBlock()->getParent();
  llvm::BasicBlock *PreloopBB = Builder.GetInsertBlock();

  // Create an alloca for the variable in the entry block
  llvm::AllocaInst *Alloca = CreateEntryBlockAlloca(TheFunction, VarName);

  // Emit the start value first, without 'variable' in scope.
  llvm::Value *StartVal = Start->codegen();
  if (!StartVal)
    return nullptr;

  // Store the value in the alloca
  Builder.CreateStore(StartVal, Alloca);

  // Make a basic block for the loop.
  llvm::BasicBlock *LoopBB =
      llvm::BasicBlock::Create(TheContext, "loop", TheFunction);

  // Create an explicit fallthrough from the preloop BB to the loop BB.
  Builder.CreateBr(LoopBB);

  // Start inserting in the loop BB.
  Builder.SetInsertPoint(LoopBB);

  // Update the symbol table in the loop, so that 'VarName' refers to this
  // alloca node. Save the old value, so we can restore it later.
  llvm::AllocaInst *OldValue = NamedValues[VarName];
  NamedValues[VarName] = Alloca;

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

  // Evaluate the loop condition.
  llvm::Value *CondValue = Cond->codegen();
  if (!CondValue)
    return nullptr;

  // Reload, increment, and restore the alloca. This handles the case where the
  // body of the loop mutates the variable.
  llvm::Value *CurValue = Builder.CreateLoad(Alloca);
  llvm::Value *NextValue = Builder.CreateFAdd(CurValue, StepValue, "nextvar");
  Builder.CreateStore(NextValue, Alloca);

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

  // Restore the symbol-table mapping.
  if (OldValue)
    NamedValues[VarName] = OldValue;
  else
    NamedValues.erase(VarName);

  // A for expression always returns 0.
  return llvm::Constant::getNullValue(llvm::Type::getDoubleTy(TheContext));
}

llvm::Value *UnaryExprAST::codegen() {
  llvm::Value *OperandV = Operand->codegen();
  if (!OperandV)
    return nullptr;

  llvm::Function *F = getFunction(std::string("unary") + Op);
  if (!F)
    return LogErrorV("Unknown unary operator");

  return Builder.CreateCall(F, OperandV, "unop");
}

llvm::Value *BinaryExprAST::codegen() {
  // Special case '=': we don't want to emit the LHS as an expression!
  if (Op == '=') {
    // Assignment requires the LHS to be an identifier.
    // NOTE: This should be a dynamic_cast, but we are building with
    // -fno-rtti...
    VariableExprAST *LHSE = static_cast<VariableExprAST *>(LHS.get());
    if (!LHSE)
      return LogErrorV("destination of '=' must be a variable");

    // Codegen the RHS.
    llvm::Value *Val = RHS->codegen();
    if (!Val)
      return nullptr;

    // Look up the name of the variable.
    llvm::Value *Variable = NamedValues[LHSE->getName()];
    if (!Variable)
      return LogErrorV("Unknown variable name");

    Builder.CreateStore(Val, Variable);
    return Val;
  }

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
    // If it wasn't a builtin binary operator, it must be a user defined one.
    // Emit a call to it.
    llvm::Function *F = getFunction(std::string("binary") + Op);
    assert(F && "binary operator not found!");

    llvm::Value *Operands[2] = {L, R};
    return Builder.CreateCall(F, Operands, "binop");
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

  // If this is an operator, install it.
  if (P.isBinaryOp())
    BinopPrecedence[P.getOperatorName()] = P.getPrecedence();

  // Create a basic block to insert instructions into
  llvm::BasicBlock *BB =
      llvm::BasicBlock::Create(TheContext, "entry", TheFunction);
  Builder.SetInsertPoint(BB);

  // Record the function arguments in the NamedValues map.
  NamedValues.clear(); // clear arguments of previous function
  for (auto &Arg : TheFunction->args()) {
    // Create an alloca for this variable.
    llvm::AllocaInst *Alloca = CreateEntryBlockAlloca(
        TheFunction, static_cast<std::string>(Arg.getName()));

    // Store the initial value in this alloca.
    Builder.CreateStore(&Arg, Alloca);

    // Add arguments to the variable symbol table.
    NamedValues[std::string(Arg.getName())] = Alloca;
  }

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

  // Promote allocas to registers.
  TheFPM->add(llvm::createPromoteMemoryToRegisterPass());

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
  BinopPrecedence['='] = 2;
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
