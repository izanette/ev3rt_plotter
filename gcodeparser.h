#pragma once

#include <stdio.h>

class Command;

class GCodeParser
{
public:
    GCodeParser(const char* inputFilePath);

    ~GCodeParser();

    Command* getNextCommand();

private:
    bool getLine(char* buf, int maxLength);
    void skipComment(char& c, char* line, int& pos);
    bool getNextWord(char* buf, int maxLength, char* line, int& pos);

private:
    FILE * m_inputFile;
};
