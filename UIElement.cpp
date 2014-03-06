#include "UIElement.hpp"

using namespace std;


UIElement::UIElement(int Y, int X, std::string Name)
{
	this->X = X;
	this->Y = Y;
	this->Name = Name;
	this->updateNeeded = true;
	this->stringValue = "---";
	this->attributes = A_NORMAL;
	this->borderVisible = false;
	this->maxWidth = 6;
	this->padding = 0;
	
	active = true;
	visible = true;
	selected = false;
}

UIElement::UIElement(int Y, int X, std::string Name, std::string Text)
{
	this->X = X;
	this->Y = Y;
	this->Name = Name;
	this->updateNeeded = true;
	this->stringValue = "---";
	this->attributes = A_NORMAL;
	this->borderVisible = false;
	this->maxWidth = 6;
	this->padding = 0;
		
	active = true;
	visible = true;
	selected = false;
	
	this->setText(Text);
}

bool UIElement::isUpdateNeeded()
{
	return updateNeeded;
}

void UIElement::update(WINDOW * window)
{
	if (visible)
	{
		stringstream displayString;
		displayString << stringValue;
		for (int i=stringValue.length();i<maxWidth;i++) displayString << " ";

		wattron(window,attributes);
		mvwprintw(window,Y,X,"%s",displayString.str().c_str());
		wattroff(window,attributes);
	}
	
	updateNeeded = false;
}

void UIElement::setSelected(bool selected)
{
	this->selected = selected;
	updateAttributes();
}

void UIElement::setActive(bool active)
{
	this->active = active;
	selected = false;
	updateAttributes();
}

void UIElement::setVisible(bool visible)
{
	this->visible = visible;
	updateNeeded = true;
}

bool UIElement::isSelected()
{
	return selected;
}

bool UIElement::isActive()
{
	return active;
}

bool UIElement::isVisible()
{
	return visible;
}

void UIElement::updateAttributes()
{
	int attr;
	if (active)
	{
		if (selected)
			attr = A_STANDOUT;
		else
			attr = A_NORMAL;
	}
	else
	{
		attr = A_DIM;
	}
	setAttributes(attr);
}

void UIElement::setAttributes(int attr)
{
	if (this->attributes != attr)
	{
		this->attributes = attr;
		updateNeeded = true;
	}
}

void UIElement::setBorderVisible(bool borderVisible)
{
	this->borderVisible = borderVisible;
}

void  UIElement::setText(string value)
{
	stringstream ss;
	if (borderVisible)
		ss << "|" << value << "|";
	else
		ss << value;
	
	this->stringValue = ss.str();
	updateNeeded = true;
}
bool UIElement::handleInput(WINDOW * inputWindow, int c) { return false;}
bool UIElement::handleGlobalInput(int c) { return false;}


int UIElement::getX() { return X;}
int UIElement::getY() { return Y;}

void UIElement::setPadding(int padding)
{
	this->padding = padding;
}

int UIElement::getPadding()
{
	return padding;
}


void UIElement::setWidth(int width)
{
	this->maxWidth = width;
}

int UIElement::getWidth()
{
	return maxWidth;
}


void NumberBox::acceptEnteredNumber()
{
	if (expectingNumbers)
	{
		double v;
		numberStream >> v;
		if (!numberStream.fail())
		{
			setValue(v);
		}
		updateNeeded = true;
		expectingNumbers = false;
		numberStream.clear();
	}
}

bool NumberBox::handleInput(WINDOW * inputWindow, int c)
{
	bool handled = true;
		
	if ((c >= '0' && c <= '9') || c == '.' || c == '-')
	{
		if (expectingNumbers)
		{
			wattron(inputWindow,attributes);
			waddch(inputWindow,c);
			wattroff(inputWindow,attributes);

			numberStream << (char)c;
		}
		else
		{
			int xOffset = (borderVisible) ? 2 : 1;
			if (c == '-' || c == '+') xOffset -= 1;

			wattron(inputWindow,attributes);
			mvwaddch(inputWindow,Y, X+xOffset, c);
			wattroff(inputWindow,attributes);

			expectingNumbers = true;
			numberStream << (char)c;
		}
	}
	else if (c == KEY_ENTER ||  c == 10)
	{
		acceptEnteredNumber();
	}
	else
	{
		switch (c)
		{
			case '+':
				setValue(getValue()+increment);
				break;
			case '-':
				setValue(getValue()-increment);
				break;
			default:
				handled = false;
				break;
		}
	}
	
	
	return handled;
}

void NumberBox::setPrecision(int precision)
{
	this->precision = precision;
	updateNeeded = true;
}

void NumberBox::setIncrement(double increment)
{
	this->increment = increment;
	updateNeeded = true;
}

void NumberBox::setSelected(bool selected)
{
	if (!selected && this->isSelected())
	{
		acceptEnteredNumber();
	}
	UIElement::setSelected(selected);
}

void NumberBox::setValue(double value)
{
	if (this->numValue != value || updateNeeded)
	{
		this->numValue = value;
		
		stringstream ss;

		ss.precision(precision);
	//	ss.width(maxWidth);
		ss.flags(ios::fixed | ios::showpos);

		double formatScale = pow(10.0,-precision);
				
		double formatted =  round(numValue/formatScale)*formatScale;
		
		if (borderVisible)
			ss << "|" << formatted << "|";
		else
			ss << formatted;
		
		this->stringValue = ss.str();
		updateNeeded = true;
	}
}


double NumberBox::getValue()
{
	return numValue;
}













