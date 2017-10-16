#include "../headers/error.h"

void errorExit(string err, int limit, ErrCode code)
{
	switch (code)
	{
		case LIMIT:
		{
			cout << err << " : valid size between 1 - " << limit << endl;
			break;
		}
		case MEMORY:
		{
			cout << err << " : invalid memory access; valid indices -> 0 ... " << (limit-1) << endl;
			break;
		}
		default:
		{
			cout << "Invalid error code" << endl;
			break;
		}
	}

	system("PAUSE");
	exit(EXIT_FAILURE);
}

// Check for invalid memory access
void checkIdx(string buffer, int idx, size_t limit)
{
	if ((idx >= limit) || (idx < 0))
	{
		errorExit(buffer, static_cast<int>(limit), MEMORY);
	}
}

// Check if the filename points to a valid .x file
void checkFileModel(string filename)
{
	ifstream file(filename);

	if ((!file.is_open()) || (file.peek() == ifstream::traits_type::eof()))
	{
		cout << "Provide input model file with .x extension and " << endl;
		cout << "make sure it's not empty." << endl;
		system("PAUSE");
		exit(EXIT_FAILURE);
	}
}

// Check if the filename points to a valid .txt file
void checkFileConfig(string filename)
{
	ifstream file(filename);

	if ((!file.is_open()) || (file.peek() == ifstream::traits_type::eof()))
	{
		cout << "Provide config file with .txt extension and " << endl;
		cout << "make sure it's not empty." << endl;
		system("PAUSE");
		exit(EXIT_FAILURE);
	}
}
