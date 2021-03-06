#include "../headers/Interpreter.h"
#include "../headers/Exception.h"
#include <unordered_map>

using namespace Eigen;

/**
 * @desc Construtor
 *
 * @param string fileName nome do arquivo a ser lido a entrada
 * @returns Interpreter
 */
Interpreter::Interpreter(const string fileName) {
    //abre o arquivo
    this->in.open(fileName.c_str());

    if(!this->in.is_open()) {
        throw(new Exception("Interpreter: Nao foi possivel abrir o arquivo de entrada!"));
    }

    if(fileName.length() > 3 && fileName.substr(fileName.length()-3, fileName.length()) != ".lp") {
        this->readFile();
    } else {
        this->readLPFile();
    }

    this->pli = new Problem(this->objectiveFunction, this->constraints, this->relations);

    this->in.close();
}

/**
 * @desc Método para ler o arquivo no formato livre
 *
 * @returns void
 */
void Interpreter::readFile() {
    long long lineNumber = 1;
    string line;
    ostringstream ss;

    //lê a primeira linha
    getline(this->in,line);

    //interpreta a primeira linha que contém a função objetivo e modo
    this->getObjectiveAndMode(line);

    //lê o restante das linhas contendo as restrições
    while(getline(in,line)) {
        //realoca a matriz de restrição e vetor de relação para receber a nova restrição
        this->constraints.conservativeResize(lineNumber,this->objectiveFunction.rows()+1);
        this->constraints.row(this->constraints.rows()-1) = VectorXd::Zero(this->objectiveFunction.rows()+1);
        this->relations.conservativeResize(lineNumber);
        //interpreta a linha e busca a restrição
        if(!this->getConstraint(line)) {
            ss << lineNumber+1;
            string message = "Interpreter: Erro na linha " + ss.str() + "!";
            throw(new Exception(message));
        }
        lineNumber++;
    }
}

/**
 * @desc Método para encontrar função objetivo e modo na string passada como parâmetro
 *
 * @param string line a primeira linha do arquivo contendo a função objetiva e o modo
 * @returns void
 */
void Interpreter::getObjectiveAndMode(string line) {
    string temp;
    long long counter, i;
    double variable;
    unordered_map<long long, double> variables;
    char op = '+';
    bool waitDigit = true;
    bool waitX = true;
    bool waitOperator = false;
    bool waitPos = false;

    //remove espaços
    line.erase(std::remove(line.begin(),line.end(),' '),line.end());

    temp = line.substr(0,3);

    //transforma para maiusculo
    transform(temp.begin(), temp.end(), temp.begin(), ::toupper);

    if(temp == "MAX") {
        this->mode = 2;
    } else if(temp == "MIN") {
        this->mode = 1;
    } else {
        throw(new Exception("Interpreter: Nao encontrado modo (Maximizar ou Minimizar) da fun��o objetivo!"));
    }

    //começa da posição 6, pois se assume as 5 primeiras sendo MINZ= ou MAXZ=
    counter = 5;
    variable = 1;

    if(line[counter]=='-') {
        op = '-';
        counter++;
        variable = -variable;
    }

    for (i=counter;i<line.length();i++){
        temp = line.substr(counter,i-counter+1);
        if(waitDigit && isdigit(line[i]) && (!isdigit(line[i+1]) && line[i+1] != '.')) {
            variable = atof(temp.c_str());
            counter=i+1;
            waitDigit = false;
            continue;
        } else if(waitX && toupper(line[i])=='X') {
            counter=i+1;
            waitX = false;
            waitDigit = false;
            waitPos = true;
            continue;
        } else if(i+1==line.length() || (waitPos && isdigit(line[i]) && !isdigit(line[i+1]))) {
            if(!waitPos || waitX) {
                throw(new Exception("Interpreter: Erro ao interpretar a funcao objetivo!"));
            }
            if(op == '-') {
               variable = -variable;
            }
            variables[atoll(temp.c_str())] = variable;
            variable = 1;
            counter=i+1;
            waitOperator = true;
            waitPos=false;
            continue;
        } else if(waitOperator && (line[i]=='+' || line[i]=='-')) {
            op = line[i];
            counter=i+1;
            waitOperator = false;
            waitDigit=true;
            waitX = true;
        }
    }

    this->objectiveFunction.resize(variables.size());

    for(i=0; i<this->objectiveFunction.rows(); i++) {
        this->objectiveFunction(i) = variables[i+1];
    }

}

 /**
 * @desc Método para buscar uma restrição da PLI a partir da string passada
 *
 * @param string line linha do arquivo contendo a restrição
 * @returns bool true se conseguiu obter a restrição
 */
bool Interpreter::getConstraint(string line) {
     string temp;
     long long counter, i;
     double variable;
     unordered_map<long long, double> variables;
     char op = '+';
     int relation;
     bool waitDigit = true;
     bool waitX = true;
     bool waitOperator = false;
     bool waitPos = false;

     //remove espaços
     line.erase(std::remove(line.begin(),line.end(),' '),line.end());

     counter = 0;
     variable = 1;

     if(line[counter]=='-') {
         op = '-';
         counter++;
         variable = -variable;
     }

     for (i=counter;i<line.length();i++){
         temp = line.substr(counter,i-counter+1);
         if(waitDigit && isdigit(line[i]) && (!isdigit(line[i+1]) && line[i+1] != '.') && i+1!=line.length()) {
             variable = atof(temp.c_str());
             counter=i+1;
             waitDigit = false;
             continue;
         } else if(waitX && toupper(line[i])=='X') {
             counter=i+1;
             waitX = false;
             waitDigit = false;
             waitPos = true;
             continue;
         } else if(waitPos && isdigit(line[i]) && !isdigit(line[i+1]) && i+1!=line.length()) {
             if(op == '-') {
                 variable = -variable;
             }
             variables[atoll(temp.c_str())] = variable;
             variable = 1;
             counter=i+1;
             waitOperator = true;
             waitPos=false;
             continue;
         } else if(waitOperator && (line[i]=='+' || line[i]=='-') && i+1!=line.length()) {
             op = line[i];
             counter=i+1;
             waitOperator = false;
             waitDigit=true;
             waitX = true;
             continue;
         } else if((line[i]=='>' || line[i]=='=' || line[i]=='<') && i+1!=line.length()) {
             if(!waitOperator) {
                 return false;
             }
             switch(line[i]) {
                 case '>':
                     relation = 1;
                     break;
                 case '=':
                     relation = 2;
                     break;
                 case '<':
                     relation = 0;
                     break;
             }
             if(line[i+1] == '=') {
                 i++;
             }
             if(line[i+2] == '-') {
                 op = '-';
                 i++;
             } else {
                 op = '+';
             }
             counter=i+1;
             waitOperator = false;
             continue;
         } else if(i+1==line.length()) {
             if(waitOperator || waitDigit || waitPos || waitX) {
                 return false;
             }
             variable = atof(temp.c_str());
             if(op == '-') {
                 variable = -variable;
             }
             variables[this->objectiveFunction.rows()+1] = variable;
         }
     }

     for(i=0; i<this->constraints.cols(); i++) {
         if(variables.find(i+1) != variables.end()) {
             this->constraints(this->constraints.rows()-1, i) = variables[i+1];
         }
     }

     this->relations(this->relations.rows()-1) = relation;

     return true;
}

/**
 * @desc Método para ler o arquivo no formato lp
 *
 * @returns void
 */
void Interpreter::readLPFile() {
    long long lineNumber = 1;
    string line;
    ostringstream ss;
    bool error, constraints = true;

    //lê a primeira linha
    getline(this->in,line);

    //interpreta a primeira e segunda linha que contém a função objetivo e modo
    this->getLPObjectiveAndMode(line);

    //a terceira linha deve ser Subject To
    getline(in,line);

    //lê o restante das linhas contendo as restrições
    while(getline(in,line)) {
        //remove espaços
        line.erase(std::remove(line.begin(),line.end(),' '),line.end());

        if(line == "Bounds") {
            constraints = false;
            lineNumber++;
            continue;
        } else if(line == "Generals") {
            break;
        }

        if(constraints){
            //realoca a matriz de restrição e vetor de relação para receber a nova restrição
            this->constraints.conservativeResize(lineNumber,this->objectiveFunction.rows()+1);
            this->constraints.row(this->constraints.rows()-1) = VectorXd::Zero(this->objectiveFunction.rows()+1);
            this->relations.conservativeResize(lineNumber);
            error = !this->getLPConstraint(line);
        } else {
            error = !this->getLPBound(line);
        }

        if(error) {
            ss << lineNumber+1;
            string message = "Interpreter: Erro na linha " + ss.str() + "!";
            throw(new Exception(message));
        }

        lineNumber++;
    }

}


/**
 * @desc Método para encontrar função objetivo e modo do formato lp
 *
 * @param string line a primeira e segunda linha do arquivo contendo o modo e a função objetiva
 * @returns void
 */
void Interpreter::getLPObjectiveAndMode(string line) {
    string temp;
    long long counter, i;
    double variable;
    unordered_map<long long, double> variables;
    char op = '+';
    bool waitDigit = true;
    bool waitX = true;
    bool waitOperator = false;
    bool waitPos = false;

    //remove espaços
    line.erase(std::remove(line.begin(),line.end(),' '),line.end());

    //transforma para maiusculo
    transform(line.begin(), line.end(), line.begin(), ::toupper);

    if(line == "MAXIMIZE") {
        this->mode = 2;
    } else if(line == "MINIMIZE") {
        this->mode = 1;
    } else {
        throw(new Exception("Interpreter: Nao encontrado modo (Maximizar ou Minimizar) da funcao objetivo!"));
    }

    //lê a segunda linha
    getline(this->in,line);

    //remove espaços
    line.erase(std::remove(line.begin(),line.end(),' '),line.end());

    //começa da posição 5, pois se assume as 4 primeiras sendo 'obj:'
    counter = 4;
    variable = 1;

    if(line[counter]=='-') {
        op = '-';
        counter++;
        variable = -variable;
    }

    for (i=counter;i<line.length();i++){
        temp = line.substr(counter,i-counter+1);
        if(waitDigit && isdigit(line[i]) && (!isdigit(line[i+1]) && line[i+1] != '.')) {
            variable = atof(temp.c_str());
            counter=i+1;
            waitDigit = false;
            continue;
        } else if(waitX && toupper(line[i])=='X') {
            counter=i+1;
            waitX = false;
            waitDigit = false;
            waitPos = true;
            continue;
        } else if(i+1==line.length() || (waitPos && isdigit(line[i]) && !isdigit(line[i+1]))) {
            if(!waitPos || waitX) {
                throw(new Exception("Interpreter: Erro ao interpretar a funcao objetivo!"));
            }
            if(op == '-') {
               variable = -variable;
            }
            variables[atoll(temp.c_str())] = variable;
            variable = 1;
            counter=i+1;
            waitOperator = true;
            waitPos=false;
            continue;
        } else if(waitOperator && (line[i]=='+' || line[i]=='-')) {
            op = line[i];
            counter=i+1;
            waitOperator = false;
            waitDigit=true;
            waitX = true;
        }
    }

    this->objectiveFunction.resize(variables.size());

    for(i=0; i<this->objectiveFunction.rows(); i++) {
        this->objectiveFunction(i) = variables[i+1];
    }
}

/**
* @desc Método para buscar uma restrição da PLI no formato lp
*
* @param string line linha do arquivo contendo a restrição, sem espaços
* @returns bool true se conseguiu obter a restrição
*/
bool Interpreter::getLPConstraint(string line) {
    string temp;
    long long counter, i;
    double variable;
    unordered_map<long long, double> variables;
    char op = '+';
    int relation;
    bool waitDigit = true;
    bool waitX = true;
    bool waitOperator = false;
    bool waitPos = false;

    //busca o valor inicial do counter
    for (i=0;i<line.length();i++){
        if(line[i] == ':') {
            break;
        }
    }

    counter = i+1;
    variable = 1;

    if(line[counter]=='-') {
        op = '-';
        counter++;
        variable = -variable;
    }

    for (i=counter;i<line.length();i++){
        temp = line.substr(counter,i-counter+1);
        if(waitDigit && isdigit(line[i]) && (!isdigit(line[i+1]) && line[i+1] != '.') && i+1!=line.length()) {
            variable = atof(temp.c_str());
            counter=i+1;
            waitDigit = false;
            continue;
        } else if(waitX && toupper(line[i])=='X') {
            counter=i+1;
            waitX = false;
            waitDigit = false;
            waitPos = true;
            continue;
        } else if(waitPos && isdigit(line[i]) && !isdigit(line[i+1]) && i+1!=line.length()) {
            if(op == '-') {
                variable = -variable;
            }
            variables[atoll(temp.c_str())] = variable;
            variable = 1;
            counter=i+1;
            waitOperator = true;
            waitPos=false;
            continue;
        } else if(waitOperator && (line[i]=='+' || line[i]=='-') && i+1!=line.length()) {
            op = line[i];
            counter=i+1;
            waitOperator = false;
            waitDigit=true;
            waitX = true;
            continue;
        } else if((line[i]=='>' || line[i]=='=' || line[i]=='<') && i+1!=line.length()) {
            if(!waitOperator) {
                return false;
            }
            switch(line[i]) {
                case '>':
                    relation = 1;
                    break;
                case '=':
                    relation = 2;
                    break;
                case '<':
                    relation = 0;
                    break;
            }
            if(line[i+1] == '=') {
                i++;
            }
            if(line[i+2] == '-') {
                op = '-';
                i++;
            } else {
                op = '+';
            }
            counter=i+1;
            waitOperator = false;
            continue;
        } else if(i+1==line.length()) {
            if(waitOperator || waitDigit || waitPos || waitX) {
                return false;
            }
            variable = atof(temp.c_str());
            if(op == '-') {
                variable = -variable;
            }
            variables[this->objectiveFunction.rows()+1] = variable;
        }
    }

    for(i=0; i<this->constraints.cols(); i++) {
        if(variables.find(i+1) != variables.end()) {
            this->constraints(this->constraints.rows()-1, i) = variables[i+1];
        }
    }

    this->relations(this->relations.rows()-1) = relation;

    return true;
}

/**
* @desc Método para buscar os limites da PLI no formato lp
*
* @param string line linha do arquivo contendo a restrição, sem espaços
* @returns bool true se conseguiu obter a restrição
*/
bool Interpreter::getLPBound(string line) {
    string temp;
    long long counter, i, pos;
    double variable;
    int relation = -1;
    bool waitDigit = true;
    bool waitX = true;
    bool waitPos = false;

    counter = 0;

    for (i=0;i<line.length();i++){
        temp = line.substr(counter,i-counter+1);
        if(waitDigit && isdigit(line[i]) && (!isdigit(line[i+1]) && line[i+1] != '.') && i+1!=line.length()) {
            variable = atof(temp.c_str());
            counter=i+1;
            waitDigit = false;
            waitX = false;
            continue;
        } else if(waitX && toupper(line[i])=='X') {
            counter=i+1;
            waitX = false;
            waitDigit = false;
            waitPos = true;
            continue;
        } else if(waitPos && isdigit(line[i]) && !isdigit(line[i+1]) && i+1!=line.length()) {
            pos = atoll(temp.c_str())-1;
            if(relation != -1 && variable > 0) {
                //realoca a matriz de restrição e vetor de relação para receber a nova restrição
                this->constraints.conservativeResize(this->constraints.rows()+1,this->objectiveFunction.rows()+1);
                this->constraints.row(this->constraints.rows()-1) = VectorXd::Zero(this->objectiveFunction.rows()+1);
                this->relations.conservativeResize(this->relations.rows()+1);
                this->constraints(this->constraints.rows()-1, pos) = 1;
                this->constraints(this->constraints.rows()-1, this->constraints.cols()-1) = variable;
                this->relations(this->relations.rows()-1) = relation;
            } else {
                relation = -2;
            }
            counter=i+1;
            waitPos=false;
            continue;
        } else if((line[i]=='>' || line[i]=='=' || line[i]=='<') && i+1!=line.length()) {
            if(relation == -1) {
                waitX = true;
            }
            switch(line[i]) {
                case '>':
                    relation = 1;
                    break;
                case '=':
                    relation = 2;
                    break;
                case '<':
                    relation = 0;
                    break;
            }
            if(line[i+1] == '=') {
                i++;
            }
            counter=i+1;
            continue;
        } else if(i+1==line.length()) {
            if(waitDigit || waitPos || waitX) {
                return false;
            }
            variable = atof(temp.c_str());
            if(variable > 0) {
                //realoca a matriz de restrição e vetor de relação para receber a nova restrição
                this->constraints.conservativeResize(this->constraints.rows()+1,this->objectiveFunction.rows()+1);
                this->constraints.row(this->constraints.rows()-1) = VectorXd::Zero(this->objectiveFunction.rows()+1);
                this->relations.conservativeResize(this->relations.rows()+1);
                this->constraints(this->constraints.rows()-1, pos) = 1;
                this->constraints(this->constraints.rows()-1, this->constraints.cols()-1) = variable;
                this->relations(this->relations.rows()-1) = relation;
            }
        }
    }

    return true;
}

/**
 * @desc Método para retornar o problema encontrado
 *
 * @returns Problem
 */
Problem* Interpreter::getProblem() {
    return this->pli;
}

/**
 * @desc Método para retornar o modo (Maximização ou Minimização)
 *
 * @returns int
 */
int Interpreter::getMode() {
    return this->mode;
}
