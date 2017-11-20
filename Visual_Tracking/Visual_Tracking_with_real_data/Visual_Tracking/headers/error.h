#pragma once
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>

using namespace std;

enum ErrCode {LIMIT, MEMORY};

// Error - exit failure - object initialization over valid dimensions
void errorExit(string err, int limit, ErrCode code);

// Check for invalid memory access
void checkIdx(string buffer, int idx, size_t limit);

// Check if the filename point to a valid .x file
bool checkFileModel(string file);

// Check if the filename points to a valid .txt file
bool checkFileConfig(string filename);
