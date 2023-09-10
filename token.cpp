#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>

#include "token.h"

#ifdef _MSC_VER
#pragma warning(disable : 4996)
#endif

char* Token::toString(char* buf)
{
    switch (m_tokenType)
    {
    case CommandT: sprintf(buf, "Command");
    case ParameterT: sprintf(buf, "Parameter");
    case UnknownT: sprintf(buf, "Unknown");

    default:
        break;
    }
    return buf;
}

char* Parameter::toString(char* buf)
{
    char c = '?';
    switch (m_parameterType)
    {
    case ParameterXT: c = 'X'; break;
    case ParameterYT: c = 'Y'; break;
    case ParameterZT: c = 'Z'; break;
    case ParameterIT: c = 'I'; break;
    case ParameterJT: c = 'J'; break;
    case ParameterFT: c = 'F'; break;
    default:
        break;
    }
    sprintf(buf, "%c %f", c, m_value);
    return buf;
}

Command::~Command()
{
    for (int i = 0; i < this->getParameterNumber(); i++)
    {
        Parameter* parameter = this->getParameter(i);
        delete parameter;
    }
}

void Command::addParameter(Parameter* parameter)
{
    m_parameters.push_back(parameter);
}

int Command::getParameterTypeIndex(EParameterType parameterType)
{
    for (int i = 0; i < this->getParameterNumber(); i++)
    {
        Parameter* parameter = this->getParameter(i);
        if (parameter->getParameterType() == parameterType)
            return i;
    }
    return -1;
}

char* Command::toString(char* buf)
{
    int i = 0;
    char c = 'G';
    switch (m_commandType)
    {
    case CommandG00T: i = 0;  c = 'G'; break;
    case CommandG01T: i = 1;  c = 'G'; break;
    case CommandG02T: i = 2;  c = 'G'; break;
    case CommandG03T: i = 3;  c = 'G'; break;
    case CommandG21T: i = 21; c = 'G'; break;
    case CommandM02T: i = 2;  c = 'M'; break;
    case CommandM03T: i = 3;  c = 'M'; break;
    case CommandM05T: i = 5;  c = 'M'; break;
    default:
        break;
    }
    sprintf(buf, "Cmd %c%02d", c, i);
    return buf;
}

Token* TokenFactory::build(char* word)
{
    char c = toupper(word[0]);

    switch (c)
    {
    case 'G': return buildCommandG(word);
    case 'M': return buildCommandM(word);

    case 'X': return buildParameter(word, ParameterXT);
    case 'Y': return buildParameter(word, ParameterYT);
    case 'Z': return buildParameter(word, ParameterZT);
    case 'I': return buildParameter(word, ParameterIT);
    case 'J': return buildParameter(word, ParameterJT);
    case 'F': return buildParameter(word, ParameterFT);

    default:
        return new Token(Token::UnknownT);
    }
}

Token* TokenFactory::buildCommandG(char* word)
{
    ECommandType commandType;
    int i = atoi(word + 1);
    switch (i)
    {
    case 0: commandType = CommandG00T; break;
    case 1: commandType = CommandG01T; break;
    case 2: commandType = CommandG02T; break;
    case 3: commandType = CommandG03T; break;
    case 21: commandType = CommandG21T; break;
    default:
        return new Token(Token::UnknownT);
    }
    return new Command(commandType);
}

Token* TokenFactory::buildCommandM(char* word)
{
    ECommandType commandType;
    int i = atoi(word + 1);
    switch (i)
    {
    case 2: commandType = CommandM02T; break;
    case 3: commandType = CommandM03T; break;
    case 5: commandType = CommandM05T; break;
    default:
        return new Token(Token::UnknownT);
    }
    return new Command(commandType);
}

Token* TokenFactory::buildParameter(char* word, EParameterType parameterType)
{
    float f = atof(word + 1);

    return new Parameter(parameterType, f);
}

