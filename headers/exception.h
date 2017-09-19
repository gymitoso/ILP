#pragma once

#include <string>
#include <iostream>

using namespace std;

/**
 * Classe para tratamento de exceção
 */
class Exception {
    private:
        string error_msg;
        unsigned long error_code;

    public:
        Exception(string error_msg);
        Exception(unsigned long error_code, string error_msg);
        void print();
        unsigned long getErrorCode();
        string getMessage();
};
