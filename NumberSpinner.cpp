#include "UIElement.hpp"

using namespace std;

NumberSpinner::NumberSpinner(int x, int y, string name, int upChar, int downChar) :
	NumberBox(x,y,name)
{
	this->upKey = upChar;
	this->downKey = downChar;
}


void NumberSpinner::update(WINDOW * window)
{
	if (visible)
	{
		mvwprintw(window,Y,X,"%c",upKey);
		attron(attributes);
		mvwprintw(window,Y+1,X,"%s",stringValue.c_str());
		attroff(attributes);
		mvwprintw(window,Y+2,X,"%c",downKey);
	}
	
	updateNeeded = false;
}

bool NumberSpinner::handleGlobalInput(int c)
{
	bool handled = false;
	if (c == upKey)
	{
		setValue(getValue()+increment);
		handled = true;
	}
	else if (c == downKey)
	{
		setValue(getValue()-increment);
		handled = true;
	}
	return handled;
}

bool NumberSpinner::handleInput(WINDOW * inputWindow, int c)
{
	bool handled = NumberBox::handleInput(inputWindow, c);

	if (!handled)
	{
		handleGlobalInput(c);
	}
	
	return handled;
}