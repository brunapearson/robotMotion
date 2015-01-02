#include <iostream>
#include <stdio.h>

class Car
{
    public:
    Car(){ std::cout << "Car's constructor" << std::endl; }
};

class Bar : public Car
{
    public:
    Bar() { std::cout << "Bar's constructor" << std::endl; }
};

