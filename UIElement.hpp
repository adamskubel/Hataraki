#ifndef HATARKAI_BASICMOTION_UI_IKELEMENT_HPP_
#define HATARKAI_BASICMOTION_UI_IKELEMENT_HPP_

#include <ncurses.h>
#include <string>
#include <sstream>
#include <cmath>

class UIElement {
	
public:
	int X,Y;
	std::string Name;
	
	UIElement(int Y, int X, std::string Name);
	UIElement(int Y, int X, std::string Name, std::string Text);

	void setText(std::string value);
	
	virtual void update(WINDOW * window);
	
	virtual void setSelected(bool enabled);
	virtual void setActive(bool active);
	virtual void setVisible(bool visible);
	
	bool isSelected();
	bool isActive();
	bool isVisible();
	
	int getY();
	int getX();
	
	void setPadding(int padding);
	int getPadding();

	void setWidth(int width);
	int getWidth();

	bool isUpdateNeeded();
	
	void setBorderVisible(bool borderVisible);
	
	virtual bool handleInput(WINDOW * inputWindow, int c);
	virtual bool handleGlobalInput(int c);
	
protected:
	bool active, visible, selected;
	bool updateNeeded, borderVisible;
	int maxWidth, padding;

	int attributes;
		
	std::string stringValue;
	
	void setAttributes(int attr);
	void updateAttributes();
};

class NumberBox : public UIElement {
	
public:
	NumberBox(int Y, int X, std::string Name) :
		UIElement(Y,X,Name)
	{
		this->precision = 2;
		this->numValue = 0;
		this->borderVisible = true;
		this->increment =  0.1;
	}
	
	void setValue(double value);
	void setPrecision(int precision);
	void setIncrement(double increment);
	
	void setSelected(bool selected);
	
	double getValue();
	
	virtual bool handleInput(WINDOW * inputWindow, int c);
	
protected:
	double numValue, increment;
	int precision;
	bool expectingNumbers;
	int startCharacter;
	std::stringstream numberStream;
	
	void acceptEnteredNumber();
	
	
};

class NumberSpinner : public NumberBox {

public:
	NumberSpinner(int y, int x, std::string name, int upKey, int downKey);
	
	bool handleInput(WINDOW * inputWindow, int c);
	bool handleGlobalInput(int c);
	void update(WINDOW * window);

private:
	int upKey, downKey;
	
	
};

#endif






















	