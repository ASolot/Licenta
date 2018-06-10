#ifndef __MENU_H__
#define __MENU_H__

#include <Arduino.h>

#define MENU_ITEMS (5)


class MenuItem
{
  public: 
	MenuItem();
	MenuItem(String description, int value);

	String getDescription();
	String getValueDescription();
	int getValue();
	void setValue(int value);

	~MenuItem();

  private: 

  	int m_value;
  	String m_description;

  	
};

class Menu
{
  public:
  	Menu();

  	void addMenuItem(MenuItem* item);
  	uint8_t getNumberOfItems();
    void incrementSelectedItem();
    void decrementSelectedItem();
    void action();
    uint8_t getSelectedItemIndex();

  	MenuItem* m_menuItemsList[MENU_ITEMS];

  	~Menu();


  private:
  	
  	uint8_t numberOfItems;
    uint8_t selectedItem;

};



#endif
