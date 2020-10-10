#include <iostream>
#include <string>

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

int main(int argc, char *argv[]) {
  int tok;

  while ((tok = gettok()) != EOF) {
    std::cout << "token: " << tok << std::endl;
  }

  return 0;
}
