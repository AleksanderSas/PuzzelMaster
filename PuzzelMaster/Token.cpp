#include "Token.h"

Token::Token()
{
	Token::counter++;
}

Token::~Token()
{
	Token::counter--;
}

int Token::counter = 0;
