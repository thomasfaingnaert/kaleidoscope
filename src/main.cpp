#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <vector>

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
};

// Numeric literals, such as "1.0"
class NumberExprAST : public ExprAST {
  double Val;

public:
  NumberExprAST(double Val) : Val(Val) {}
};

// Variable expression, i.e. a reference to a variable
class VariableExprAST : public ExprAST {
  std::string Name;

public:
  VariableExprAST(const std::string &Name) : Name(Name) {}
};

// Binary expression
class BinaryExprAST : public ExprAST {
  char Op;
  std::unique_ptr<ExprAST> LHS, RHS;

public:
  BinaryExprAST(char op, std::unique_ptr<ExprAST> LHS,
                std::unique_ptr<ExprAST> RHS)
      : Op(op), LHS(std::move(LHS)), RHS(std::move(RHS)) {}
};

// Function call
class CallExprAST : public ExprAST {
  std::string Callee;
  std::vector<std::unique_ptr<ExprAST>> Args;

public:
  CallExprAST(const std::string &Callee,
              std::vector<std::unique_ptr<ExprAST>> Args)
      : Callee(Callee), Args(std::move(Args)) {}
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
};

// The function definition itself
class FunctionAST {
  std::unique_ptr<PrototypeAST> Proto;
  std::unique_ptr<ExprAST> Body;

public:
  FunctionAST(std::unique_ptr<PrototypeAST> Proto,
              std::unique_ptr<ExprAST> Body)
      : Proto(std::move(Proto)), Body(std::move(Body)) {}
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

/* Parser */

std::unique_ptr<ExprAST> ParseExpression(); // forward decl

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
    auto Proto = std::make_unique<PrototypeAST>("", std::vector<std::string>());
    return std::make_unique<FunctionAST>(std::move(Proto), std::move(E));
  }
  return nullptr;
}

/* Top level parsing */
void HandleDefinition() {
  if (ParseDefinition()) {
    std::cerr << "Parsed a function definition.\n";
  } else {
    getNextToken(); // simple error recovery: skip to next token
  }
}

void HandleExtern() {
  if (ParseExtern()) {
    std::cerr << "Parsed an extern.\n";
  } else {
    getNextToken(); // simple error recovery: skip to next token
  }
}

void HandleTopLevelExpression() {
  if (ParseTopLevelExpr()) {
    std::cerr << "Parsed a top-level expression\n";
  } else {
    getNextToken(); // simple error recovery: skip to next token
  }
}

int main(int argc, char *argv[]) {
  // Register default binary operators.
  BinopPrecedence['<'] = 10;
  BinopPrecedence['+'] = 20;
  BinopPrecedence['-'] = 20;
  BinopPrecedence['*'] = 40;

  // Prime the first token
  std::cerr << "ready> ";
  getNextToken();

  while (true) {
    std::cerr << "ready> ";

    switch (CurTok) {
    case tok_eof:
      return 0; // end of file, stop running
    case ';':
      getNextToken(); // ignore top level ';'
      break;
    case tok_def:
      HandleDefinition();
      break;
    case tok_extern:
      HandleExtern();
      break;
    default:
      HandleTopLevelExpression();
      break;
    }
  }

  return 0;
}
