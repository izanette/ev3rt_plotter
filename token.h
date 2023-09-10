#pragma once

#include <vector>

typedef enum
{
    CommandG00T,
    CommandG01T,
    CommandG02T,
    CommandG03T,
    CommandG21T,
    CommandM02T,
    CommandM03T,
    CommandM05T
} ECommandType;

typedef enum
{
    ParameterXT,
    ParameterYT,
    ParameterZT,
    ParameterIT,
    ParameterJT,
    ParameterFT,
} EParameterType;

class Token
{
public:
    typedef enum
    {
        //CommentT,
        CommandT,
        ParameterT,
        UnknownT
    } ETokenType;

    Token(ETokenType tokenType)
        : m_tokenType(tokenType)
    {}

    virtual ~Token() {}

    ETokenType getTokenType() { return m_tokenType; }

    virtual char* toString(char* buf);

private:
    ETokenType m_tokenType;
};

class Parameter : public Token
{
public:
    Parameter(EParameterType parameterType, float value)
        : Token(Token::ParameterT)
        , m_parameterType(parameterType)
        , m_value(value)
    {}

    virtual ~Parameter() {}

    EParameterType getParameterType() { return m_parameterType; }
    float getValue() { return m_value; }
    virtual char* toString(char* buf);

private:
    EParameterType m_parameterType;
    float m_value;
};

class Command : public Token
{
public:
    Command(ECommandType commandType)
        : Token(Token::CommandT)
        , m_commandType(commandType)
    {}

    virtual ~Command();

    void addParameter(Parameter* parameter);

    ECommandType getCommandType() { return m_commandType; }
    int getParameterNumber() { return m_parameters.size(); }
    int getParameterTypeIndex(EParameterType parameterType);
    Parameter* getParameter(int i) { return m_parameters[i]; }

    virtual char* toString(char* buf);

private:
    ECommandType m_commandType;
    std::vector<Parameter*> m_parameters;
};

class TokenFactory
{
public:
    static Token* build(char* word);

private:
    static Token* buildCommandG(char* word);
    static Token* buildCommandM(char* word);
    static Token* buildParameter(char* word, EParameterType parameterType);
};
