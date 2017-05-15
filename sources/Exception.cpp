#include "exception.h"

/**
 * Construtor
 * Passa apenas uma mensagem de erro
 *
 * @param error_msg
 * @returns Exception
 */
Exception::Exception(string error_msg) {
	this->error_msg = error_msg;
}

/**
 * Construtor
 * Passa uma mensagem de erro e o código do erro
 *
 * @param error_code
 * @param error_msg
 * @returns Exception
 */
Exception::Exception(unsigned long error_code, string error_msg) {
	this->error_msg = error_msg;
	this->error_code = error_code;
}

/**
 * Mostra no console a mensagem do erro.
 */
void Exception::print() {
	cout<< this->error_msg << endl;
}

/**
 * Retorna o código do erro.
 *
 * @returns unsigned long
 */
unsigned long Exception::getErrorCode() {
	return this->error_code;
}

/**
 * Retorna a mensagem da exceção
 *
 * @returns string
 */
string Exception::getMessage() {
	return this->error_msg;
}
