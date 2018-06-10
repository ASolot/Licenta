#include "menu.h"


MenuItem::MenuItem()
{
	m_description = String("");
	m_value = 0;

}

MenuItem::MenuItem(String description, int value)
{
	m_description = description;
	m_value = value;
}

MenuItem::~MenuItem()
{
  
}

String MenuItem::getDescription()
{
	return m_description;
}

String MenuItem::getValueDescription()
{
	if(m_value == -1)
		return String("ON");
	if(m_value == -2)
		return String("OFF");

	if(m_value == 0)
		return String(" +");

	return String(m_value);
}

int MenuItem::getValue()
{
	return m_value;
}

void MenuItem::setValue(int value)
{
	m_value = value;
}

Menu::Menu()
{
	numberOfItems = 0;
  selectedItem = 0;
}

Menu::~Menu()
{
  
}

void Menu::addMenuItem(MenuItem* item)
{
	if(numberOfItems < MENU_ITEMS)
	{
		m_menuItemsList[numberOfItems] = item;
		numberOfItems++;
	}

	return;
}

uint8_t Menu::getNumberOfItems()
{
	return numberOfItems;
}

void Menu::incrementSelectedItem()
{
  selectedItem++;
  if(selectedItem == numberOfItems)
    selectedItem = 0;
}

void Menu::decrementSelectedItem()
{
  if(selectedItem == 0)
    selectedItem = numberOfItems - 1;
  else
    selectedItem--; 
 
}

uint8_t Menu::getSelectedItemIndex()
{
  return selectedItem;
}

void Menu::action()
{
  // toggle behaviour
  if(m_menuItemsList[selectedItem]->getValue() == -1)
    m_menuItemsList[selectedItem]->setValue(-2);
  else if(m_menuItemsList[selectedItem]->getValue() == -2)
    m_menuItemsList[selectedItem]->setValue(-1);
}

