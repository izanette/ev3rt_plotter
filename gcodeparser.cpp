#include <ctype.h>
#include <vector>

#include "token.h"
#include "gcodeparser.h"

#ifdef _MSC_VER
#pragma warning(disable : 4996)
#endif

GCodeParser::GCodeParser(const char* inputFilePath)
{
    m_inputFile = fopen(inputFilePath, "r");
}

GCodeParser::~GCodeParser()
{
    if (m_inputFile != 0)
        fclose(m_inputFile);
}

Command* GCodeParser::getNextCommand()
{
    const int MAX_LINE_SIZE = 256;
    char buf[MAX_LINE_SIZE];

    if (m_inputFile == 0) return NULL;

    while (getLine(buf, MAX_LINE_SIZE))
    {
        char word[32];
        int pos = 0;
        Command* cmd = NULL;
        while (getNextWord(word, 32, buf, pos))
        {
            Token* token = TokenFactory::build(word);
            switch (token->getTokenType())
            {
            case Token::CommandT:
                cmd = static_cast<Command*>(token);
                break;

            case Token::ParameterT:
                if (cmd != NULL)
                    cmd->addParameter(static_cast<Parameter*>(token));
                break;

            default:
                break;
            }
        }

        if (cmd) return cmd;
    }

    return NULL;
}

bool GCodeParser::getLine(char* buf, int maxLength)
{
    if (!m_inputFile || feof(m_inputFile)) return false;

    int sz = 0;

    char c = getc(m_inputFile);
    if (c == EOF) return false;

    while (c != '\n'
        && c != EOF
        && sz < maxLength - 1)
    {
        buf[sz] = c;
        sz++;
        c = getc(m_inputFile);
    }
    buf[sz] = '\0';

    return true;
}

void GCodeParser::skipComment(char& c, char* line, int& pos)
{
    while (c != ')' && c != 0)
        c = line[++pos];
    if (c == ')') c = line[++pos];
}

bool GCodeParser::getNextWord(char* buf, int maxLength, char* line, int& pos)
{
    char c = line[pos];
    while ((c == '(' || isspace(c)) && c != 0)
    {
        if (c == '(')
        {
            skipComment(c, line, pos);
            continue;
        }

        c = line[++pos];
    }

    int i = 0;
    while (!isspace(c) && c != 0 && i < maxLength - 1)
    {
        if (c == '(')
        {
            skipComment(c, line, pos);
            continue;
        }

        buf[i++] = c;
        c = line[++pos];
    }

    buf[i] = 0;
    return (i > 0 || c != 0);
}
