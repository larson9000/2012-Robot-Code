#ifndef DISPLAYWRITER_H
#define DISPLAYWRITER_H

#include <cstdarg>

/**
 * This wraps around the LCD Display for the driver station.
 * This 
 */
class DisplayWriter
{
public:
	DisplayWriter();
	~DisplayWriter();

	/**
	 * Clear the display buffer.
	 */
	void clear();

	/**
	 * specify the number of lines to reserve for this writer
	 * the lines are on a first-come-first-served basis
	 */
	void reserve(int size);

	/**
	 * Printf to the next line in the display, this "scrolls".
	 */
	int printf(const char* format, ...);

	/**
	 * Printf at a specific line.
	 */
	int printfLine(int line, const char* format, ...);

private:
	int startingLine;
	int reservedSize;
	int nextPrintfLine;
};

#endif // DISPLAYWRITER_H
