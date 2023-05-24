#include <iostream>
#include <fstream>
// #include <boost/functional/hash.hpp>
#include <regex>
#include <unordered_set>
#include <set>
#include <list>
#include <unordered_map>
#include <algorithm>
#include <stdexcept>
#include <math.h>
#include <cfloat>
#include <queue>
#include <unordered_map>

#define SYMBOLS 0
#define INITIAL 1
#define GOAL 2
#define ACTIONS 3
#define ACTION_DEFINITION 4
#define ACTION_PRECONDITION 5
#define ACTION_EFFECT 6

class GroundedCondition;
class Condition;
class GroundedAction;
class Action;
class Env;

using namespace std;

bool print_status = true;

class GroundedCondition
{
private:
    string predicate;
    list<string> arg_values;
    bool truth = true;

public:
    GroundedCondition(string predicate, list<string> arg_values, bool truth = true)
    {
        this->predicate = predicate;
        this->truth = truth;  // fixed
        for (string l : arg_values)
        {
            this->arg_values.push_back(l);
        }
    }

    GroundedCondition(const GroundedCondition& gc)
    {
        this->predicate = gc.predicate;
        this->truth = gc.truth;  // fixed
        for (string l : gc.arg_values)
        {
            this->arg_values.push_back(l);
        }
    }

    string get_predicate() const
    {
        return this->predicate;
    }
    list<string> get_arg_values() const
    {
        return this->arg_values;
    }

    bool get_truth() const
    {
        return this->truth;
    }

    void set_truth(bool truth)  {
        this->truth = truth;
    }

    friend ostream& operator<<(ostream& os, const GroundedCondition& pred)
    {
        os << pred.toString() << " ";
        return os;
    }

    bool operator==(const GroundedCondition& rhs) const
    {
        if (this->predicate != rhs.predicate || this->arg_values.size() != rhs.arg_values.size())
            return false;

        auto lhs_it = this->arg_values.begin();
        auto rhs_it = rhs.arg_values.begin();

        while (lhs_it != this->arg_values.end() && rhs_it != rhs.arg_values.end())
        {
            if (*lhs_it != *rhs_it)
                return false;
            ++lhs_it;
            ++rhs_it;
        }

        if (this->truth != rhs.get_truth()) // fixed
            return false;

        return true;
    }

    string toString() const
    {
        string temp = "";
        temp += this->predicate;
        temp += "(";
        for (string l : this->arg_values)
        {
            temp += l + ",";
        }
        temp = temp.substr(0, temp.length() - 1);
        temp += ")";
        return temp;
    }
};

struct GroundedConditionComparator
{
    bool operator()(const GroundedCondition& lhs, const GroundedCondition& rhs) const
    {
        return lhs == rhs;
    }
};

struct GroundedConditionHasher
{
    size_t operator()(const GroundedCondition& gcond) const
    {
        return hash<string>{}(gcond.toString());
    }
};

class Condition
{
private:
    string predicate;
    list<string> args;
    bool truth;

public:
    Condition(string pred, list<string> args, bool truth)
    {
        this->predicate = pred;
        this->truth = truth;
        for (string ar : args)
        {
            this->args.push_back(ar);
        }
    }

    string get_predicate() const
    {
        return this->predicate;
    }

    list<string> get_args() const
    {
        return this->args;
    }

    bool get_truth() const
    {
        return this->truth;
    }

    friend ostream& operator<<(ostream& os, const Condition& cond)
    {
        os << cond.toString() << " ";
        return os;
    }

    bool operator==(const Condition& rhs) const // fixed
    {

        if (this->predicate != rhs.predicate || this->args.size() != rhs.args.size())
            return false;

        auto lhs_it = this->args.begin();
        auto rhs_it = rhs.args.begin();

        while (lhs_it != this->args.end() && rhs_it != rhs.args.end())
        {
            if (*lhs_it != *rhs_it)
                return false;
            ++lhs_it;
            ++rhs_it;
        }

        if (this->truth != rhs.get_truth())
            return false;

        return true;
    }

    string toString() const
    {
        string temp = "";
        if (!this->truth)
            temp += "!";
        temp += this->predicate;
        temp += "(";
        for (string l : this->args)
        {
            temp += l + ",";
        }
        temp = temp.substr(0, temp.length() - 1);
        temp += ")";
        return temp;
    }
};

struct ConditionComparator
{
    bool operator()(const Condition& lhs, const Condition& rhs) const
    {
        return lhs == rhs;
    }
};

struct ConditionHasher
{
    size_t operator()(const Condition& cond) const
    {
        return hash<string>{}(cond.toString());
    }
};

class Action
{
private:
    string name;
    list<string> args;
    unordered_set<Condition, ConditionHasher, ConditionComparator> preconditions;
    unordered_set<Condition, ConditionHasher, ConditionComparator> effects;

public:
    Action(string name, list<string> args,
        unordered_set<Condition, ConditionHasher, ConditionComparator>& preconditions,
        unordered_set<Condition, ConditionHasher, ConditionComparator>& effects)
    {
        this->name = name;
        for (string l : args)
        {
            this->args.push_back(l);
        }
        for (Condition pc : preconditions)
        {
            this->preconditions.insert(pc);
        }
        for (Condition pc : effects)
        {
            this->effects.insert(pc);
        }
    }
    string get_name() const
    {
        return this->name;
    }
    list<string> get_args() const
    {
        return this->args;
    }
    unordered_set<Condition, ConditionHasher, ConditionComparator> get_preconditions() const
    {
        return this->preconditions;
    }
    unordered_set<Condition, ConditionHasher, ConditionComparator> get_effects() const
    {
        return this->effects;
    }

    bool operator==(const Action& rhs) const
    {
        if (this->get_name() != rhs.get_name() || this->get_args().size() != rhs.get_args().size())
            return false;

        return true;
    }

    friend ostream& operator<<(ostream& os, const Action& ac)
    {
        os << ac.toString() << endl;
        os << "Precondition: ";
        for (Condition precond : ac.get_preconditions())
            os << precond;
        os << endl;
        os << "Effect: ";
        for (Condition effect : ac.get_effects())
            os << effect;
        os << endl;
        return os;
    }

    string toString() const
    {
        string temp = "";
        temp += this->get_name();
        temp += "(";
        for (string l : this->get_args())
        {
            temp += l + ",";
        }
        temp = temp.substr(0, temp.length() - 1);
        temp += ")";
        return temp;
    }
};

struct ActionComparator
{
    bool operator()(const Action& lhs, const Action& rhs) const
    {
        return lhs == rhs;
    }
};

struct ActionHasher
{
    size_t operator()(const Action& ac) const
    {
        return hash<string>{}(ac.get_name());
    }
};

class Env
{
private:
    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> initial_conditions;
    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> goal_conditions;
    unordered_set<Action, ActionHasher, ActionComparator> actions;
    unordered_set<string> symbols;

public:
    void remove_initial_condition(GroundedCondition gc)
    {
        this->initial_conditions.erase(gc);
    }
    void add_initial_condition(GroundedCondition gc)
    {
        this->initial_conditions.insert(gc);
    }
    void add_goal_condition(GroundedCondition gc)
    {
        this->goal_conditions.insert(gc);
    }
    void remove_goal_condition(GroundedCondition gc)
    {
        this->goal_conditions.erase(gc);
    }
    void add_symbol(string symbol)
    {
        symbols.insert(symbol);
    }
    void add_symbols(list<string> symbols)
    {
        for (string l : symbols)
            this->symbols.insert(l);
    }
    void add_action(Action action)
    {
        this->actions.insert(action);
    }

    Action get_action(string name)
    {
        for (Action a : this->actions)
        {
            if (a.get_name() == name)
                return a;
        }
        throw runtime_error("Action " + name + " not found!");
    }
    unordered_set<string> get_symbols() const
    {
        return this->symbols;
    }

    auto get_initial_condition() const
    {
        return this->initial_conditions;
    }

    auto get_goal_condition() const
    {
        return this->goal_conditions;
    }

    auto get_actions() const
    {
        return this->actions;
    }

    friend ostream& operator<<(ostream& os, const Env& w)
    {
        os << "***** Environment *****" << endl << endl;
        os << "Symbols: ";
        for (string s : w.get_symbols())
            os << s + ",";
        os << endl;
        os << "Initial conditions: ";
        for (GroundedCondition s : w.initial_conditions)
            os << s;
        os << endl;
        os << "Goal conditions: ";
        for (GroundedCondition g : w.goal_conditions)
            os << g;
        os << endl;
        os << "Actions:" << endl;
        for (Action g : w.actions)
            os << g << endl;
        cout << "***** Environment Created! *****" << endl;
        return os;
    }
};

class GroundedAction
{
private:
    string name;
    list<string> arg_values;

    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> grounded_preconditions;
    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> grounded_effects;

public:
    GroundedAction(){};
    GroundedAction(string name, list<string> arg_values)
    {
        this->name = name;
        for (string ar : arg_values)
        {
            this->arg_values.push_back(ar);
        }
    }

    GroundedAction(string name, list<string> arg_values,
        unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator>& preconditions,
        unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator>& effects)
    {
        this->name = name;
        for (string ar: arg_values){
            this->arg_values.push_back(ar);
        }       
        for (GroundedCondition pc : preconditions)
        {
            this->grounded_preconditions.insert(pc);
        }
        for (GroundedCondition pc : effects)
        {
            this->grounded_effects.insert(pc);
        }
    }

    string get_name() const
    {
        return this->name;
    }

    list<string> get_arg_values() const
    {
        return this->arg_values;
    }

    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> get_preconditions() const
    {
        return this->grounded_preconditions;
    }

    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> get_effects() const
    {
        return this->grounded_effects;
    }

    bool operator==(const GroundedAction& rhs) const
    {
        if (this->name != rhs.name || this->arg_values.size() != rhs.arg_values.size())
            return false;

        auto lhs_it = this->arg_values.begin();
        auto rhs_it = rhs.arg_values.begin();

        while (lhs_it != this->arg_values.end() && rhs_it != rhs.arg_values.end())
        {
            if (*lhs_it != *rhs_it)
                return false;
            ++lhs_it;
            ++rhs_it;
        }
        return true;
    }

    friend ostream& operator<<(ostream& os, const GroundedAction& gac)
    {
        os << gac.toString() << " ";
        return os;
    }

    string toString() const
    {
        string temp = "";
        temp += this->name;
        temp += "(";
        for (string l : this->arg_values)
        {
            temp += l + ",";
        }
        temp = temp.substr(0, temp.length() - 1);
        temp += ")";
        return temp;
    }
};

list<string> parse_symbols(string symbols_str)
{
    list<string> symbols;
    size_t pos = 0;
    string delimiter = ",";
    while ((pos = symbols_str.find(delimiter)) != string::npos)
    {
        string symbol = symbols_str.substr(0, pos);
        symbols_str.erase(0, pos + delimiter.length());
        symbols.push_back(symbol);
    }
    symbols.push_back(symbols_str);
    return symbols;
}

Env* create_env(char* filename)
{
    ifstream input_file(filename);
    Env* env = new Env();
    regex symbolStateRegex("symbols:", regex::icase);
    regex symbolRegex("([a-zA-Z0-9_, ]+) *");
    regex initialConditionRegex("initialconditions:(.*)", regex::icase);
    regex conditionRegex("(!?[A-Z][a-zA-Z_]*) *\\( *([a-zA-Z0-9_, ]+) *\\)");
    regex goalConditionRegex("goalconditions:(.*)", regex::icase);
    regex actionRegex("actions:", regex::icase);
    regex precondRegex("preconditions:(.*)", regex::icase);
    regex effectRegex("effects:(.*)", regex::icase);
    int parser = SYMBOLS;

    unordered_set<Condition, ConditionHasher, ConditionComparator> preconditions;
    unordered_set<Condition, ConditionHasher, ConditionComparator> effects;
    string action_name;
    string action_args;

    string line;
    if (input_file.is_open())
    {
        while (getline(input_file, line))
        {
            string::iterator end_pos = remove(line.begin(), line.end(), ' ');
            line.erase(end_pos, line.end());

            if (line == "")
                continue;

            if (parser == SYMBOLS)
            {
                smatch results;
                if (regex_search(line, results, symbolStateRegex))
                {
                    line = line.substr(8);
                    sregex_token_iterator iter(line.begin(), line.end(), symbolRegex, 0);
                    sregex_token_iterator end;

                    env->add_symbols(parse_symbols(iter->str()));  // fixed

                    parser = INITIAL;
                }
                else
                {
                    cout << "Symbols are not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == INITIAL)
            {
                const char* line_c = line.c_str();
                if (regex_match(line_c, initialConditionRegex))
                {
                    const std::vector<int> submatches = { 1, 2 };
                    sregex_token_iterator iter(
                        line.begin(), line.end(), conditionRegex, submatches);
                    sregex_token_iterator end;

                    while (iter != end)
                    {
                        // name
                        string predicate = iter->str();
                        iter++;
                        // args
                        string args = iter->str();
                        iter++;

                        if (predicate[0] == '!')
                        {
                            env->remove_initial_condition(
                                GroundedCondition(predicate.substr(1), parse_symbols(args)));
                        }
                        else
                        {
                            env->add_initial_condition(
                                GroundedCondition(predicate, parse_symbols(args)));
                        }
                    }

                    parser = GOAL;
                }
                else
                {
                    cout << "Initial conditions not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == GOAL)
            {
                const char* line_c = line.c_str();
                if (regex_match(line_c, goalConditionRegex))
                {
                    const std::vector<int> submatches = { 1, 2 };
                    sregex_token_iterator iter(
                        line.begin(), line.end(), conditionRegex, submatches);
                    sregex_token_iterator end;

                    while (iter != end)
                    {
                        // name
                        string predicate = iter->str();
                        iter++;
                        // args
                        string args = iter->str();
                        iter++;

                        if (predicate[0] == '!')
                        {
                            env->remove_goal_condition(
                                GroundedCondition(predicate.substr(1), parse_symbols(args)));
                        }
                        else
                        {
                            env->add_goal_condition(
                                GroundedCondition(predicate, parse_symbols(args)));
                        }
                    }

                    parser = ACTIONS;
                }
                else
                {
                    cout << "Goal conditions not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == ACTIONS)
            {
                const char* line_c = line.c_str();
                if (regex_match(line_c, actionRegex))
                {
                    parser = ACTION_DEFINITION;
                }
                else
                {
                    cout << "Actions not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == ACTION_DEFINITION)
            {
                const char* line_c = line.c_str();
                if (regex_match(line_c, conditionRegex))
                {
                    const std::vector<int> submatches = { 1, 2 };
                    sregex_token_iterator iter(
                        line.begin(), line.end(), conditionRegex, submatches);
                    sregex_token_iterator end;
                    // name
                    action_name = iter->str();
                    iter++;
                    // args
                    action_args = iter->str();
                    iter++;

                    parser = ACTION_PRECONDITION;
                }
                else
                {
                    cout << "Action not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == ACTION_PRECONDITION)
            {
                const char* line_c = line.c_str();
                if (regex_match(line_c, precondRegex))
                {
                    const std::vector<int> submatches = { 1, 2 };
                    sregex_token_iterator iter(
                        line.begin(), line.end(), conditionRegex, submatches);
                    sregex_token_iterator end;

                    while (iter != end)
                    {
                        // name
                        string predicate = iter->str();
                        iter++;
                        // args
                        string args = iter->str();
                        iter++;

                        bool truth;

                        if (predicate[0] == '!')
                        {
                            predicate = predicate.substr(1);
                            truth = false;
                        }
                        else
                        {
                            truth = true;
                        }

                        Condition precond(predicate, parse_symbols(args), truth);
                        preconditions.insert(precond);
                    }

                    parser = ACTION_EFFECT;
                }
                else
                {
                    cout << "Precondition not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == ACTION_EFFECT)
            {
                const char* line_c = line.c_str();
                if (regex_match(line_c, effectRegex))
                {
                    const std::vector<int> submatches = { 1, 2 };
                    sregex_token_iterator iter(
                        line.begin(), line.end(), conditionRegex, submatches);
                    sregex_token_iterator end;

                    while (iter != end)
                    {
                        // name
                        string predicate = iter->str();
                        iter++;
                        // args
                        string args = iter->str();
                        iter++;

                        bool truth;

                        if (predicate[0] == '!')
                        {
                            predicate = predicate.substr(1);
                            truth = false;
                        }
                        else
                        {
                            truth = true;
                        }

                        Condition effect(predicate, parse_symbols(args), truth);
                        effects.insert(effect);
                    }

                    env->add_action(
                        Action(action_name, parse_symbols(action_args), preconditions, effects));

                    preconditions.clear();
                    effects.clear();
                    parser = ACTION_DEFINITION;
                }
                else
                {
                    cout << "Effects not specified correctly." << endl;
                    throw;
                }
            }
        }
        input_file.close();
    }

    else
        cout << "Unable to open file";

    return env;
}

// -----------------------------------------------------------------------------

class node{
    public:
        size_t self_hash;
        size_t parent_hash;
        double f = FLT_MAX;
        double g = FLT_MAX;
        double h = FLT_MAX;
        std::unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> state;
        GroundedAction action;
};

size_t nodeHasher(node input_node){
    string temp_string = "";
    for(auto &it : input_node.state){
        temp_string += it.toString();
    }
    return hash<string>{}(temp_string);
}

bool node_comparator(node a, node b){
    return(a.f > b.f);
}

double calc_Heuristic(node ref_node, node goal_node){
    double h = goal_node.state.size();
    for(auto & it : ref_node.state){
        for (auto & itt : goal_node.state){
            if(it==itt){
                h--;
            }
        }
    }
    return h;
}

void getCombs(std::vector<std::string> symbols, int req_len, int s, int curr_len, std::vector<bool> check, int l, std::vector<std::list<std::string>> &combinations){
    std::list<std::string> temp;
    if (curr_len > req_len){
        return;
    }
    else if (curr_len == req_len){
        temp.clear();
        for (int i = 0; i<l; i++){
            if (check[i]==true){
                temp.push_back(symbols[i]);
            }
        }
        combinations.push_back(temp);
        return;
    }
    if (s==l){
        return;
    }
    check[s] = true;
    getCombs(symbols, req_len, s+1, curr_len+1, check, l, combinations);
    check[s] = false;
    getCombs(symbols, req_len, s+1, curr_len, check, l, combinations);
}

void getPerms(std::vector<std::list<std::string>> &combinations, std::vector<std::list<std::string>> &permutations){

    for (auto &it : combinations){
        it.sort();
        do {
            permutations.push_back(it);
        }while(std::next_permutation(it.begin(), it.end()));
    }

}


void getActions(const unordered_set<Action, ActionHasher, ActionComparator> &actions, const unordered_set<string> &all_symbols, std::list<GroundedAction>&all_actions){
    std::vector<std::string> symbols (all_symbols.begin(), all_symbols.end());
    std::vector<bool> check (symbols.size(), false);
    std::vector<std::list<std::string>> combinations;

    for(int i=1;i<=symbols.size();i++){
        getCombs(symbols, i, 0, 0, check, symbols.size(), combinations);
    }

    std::vector<std::list<std::string>> permutations;
    getPerms(combinations, permutations);

    for(const auto &action: actions){
        for(const auto &permutation: permutations){
            if(action.get_args().size() == permutation.size()){
                                
                auto preconditions = action.get_preconditions();
                auto effects = action.get_effects();
                auto arg_names = action.get_args();

                unordered_map<string, string> string_map;

                unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> grounded_preconditions;
                unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> grounded_effects;

                for(auto& symbol: all_symbols){
                    string_map[symbol] = symbol;
                }

                auto it = permutation.begin();
                for(auto& arg: arg_names){
                    string_map[arg] = *it;
                    it++;
                }

                for(auto& effect: effects){
                    list<string> grounded_args;
                    for(auto& arg: effect.get_args()){
                        grounded_args.push_back(string_map[arg]);
                    }
                    GroundedCondition grounded_effect(effect.get_predicate(), grounded_args, effect.get_truth());
                    grounded_effects.insert(grounded_effect);
                }

                for(auto& condition: preconditions){
                    list<string> grounded_args;
                    for(auto& arg: condition.get_args()){
                        grounded_args.push_back(string_map[arg]);
                    }
                    GroundedCondition grounded_condition(condition.get_predicate(), grounded_args, condition.get_truth());
                    grounded_preconditions.insert(grounded_condition);
                }

                GroundedAction grounded_action(action.get_name(), permutation, grounded_preconditions, grounded_effects);
                all_actions.push_back(grounded_action);
            }
        }
    }
}

list<GroundedAction> planner(Env* env)
{

    clock_t start_time = clock();
    bool in_closedlist = false;
    bool is_valid_action = false;
    // initialize start node
    node start_node;
    start_node.state = env->get_initial_condition();
    start_node.parent_hash = -1;
    start_node.self_hash = nodeHasher(start_node);
    start_node.g = 0;

    // initialize goal node
    node goal_node;
    goal_node.state = env->get_goal_condition();
    goal_node.self_hash = nodeHasher(goal_node);

    // computing all the possible actions
    std::list<GroundedAction>all_actions;
    std::unordered_set<std::string> all_symbols = env->get_symbols();
    std::unordered_set<Action, ActionHasher, ActionComparator> basic_actions = env->get_actions();

    getActions(basic_actions, all_symbols, all_actions);


    // std::cout<<"Actions initialised"<<std::endl;

    std::priority_queue<node, std::vector<node>, decltype(&node_comparator)> openlist(node_comparator);
    unordered_map<size_t, node>closedlist; // stores the self_hash and the node
    unordered_map<size_t, node>hash_table; // stores the hashes and the node, will be used for backtracking

    openlist.push(start_node);
    hash_table[start_node.self_hash] = start_node;
    size_t goal_node_hash;

    node to_expand;

    while(!openlist.empty()){

        to_expand = openlist.top();
        openlist.pop();

        if(hash_table[to_expand.self_hash].h == 0){
            std::cout<<"Goal Found!!"<<std::endl;
            goal_node_hash = to_expand.self_hash;
            break;
        }

        if(closedlist.find(to_expand.self_hash)!=closedlist.end()){
            // already in closed list, ignore and evaluate next node
            continue;
        }
        closedlist[to_expand.self_hash] = to_expand;

        for(auto &action : all_actions){
            unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> preconditions = action.get_preconditions();
            unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> effects = action.get_effects();
            is_valid_action = true;
            for(auto precondition : preconditions){
                // std::cout<<"  searching: "<<precondition<<std::endl;
                if (to_expand.state.find(precondition)==to_expand.state.end()){
                    is_valid_action = false;
                    break;
                }
            }

            if(is_valid_action){
                // std::cout<<"***valid action found***"<<std::endl;
                node child_node;
                child_node.state = to_expand.state;

                for(auto effect: effects){
                    if(effect.get_truth()){
                        child_node.state.insert(effect);
                    }
                    else{
                        effect.set_truth(true);
                        child_node.state.erase(effect);
                    }
                }

                // child node created
                child_node.self_hash = nodeHasher(child_node);
                in_closedlist = false;
                if(closedlist.find(child_node.self_hash)!=closedlist.end()){
                    // inclosed list, ignore and continue
                    in_closedlist = true;
                }

                if(!in_closedlist){

                    double g_val = to_expand.g + 1;
                    // std::cout<<child_node.g<<" "<<to_expand.g<<std::endl;
                    if(child_node.g > g_val){
                        child_node.g = g_val;
                        child_node.h = calc_Heuristic(child_node, goal_node);
                        // child_node.h = 0;
                        child_node.f = child_node.g + child_node.h;
                        // child_node.f = child_node.g;
                        child_node.parent_hash = to_expand.self_hash;
                        child_node.action = action;
                        hash_table[child_node.self_hash] = child_node;
                        openlist.push(child_node);
                        // std::cout<<"openlist size: "<<openlist.size()<<std::endl;

                    }

                }

            }

        }          

    }

    size_t curr_hash = goal_node_hash;
    std::list<GroundedAction>actions;
    // int flag = 0;
    while(curr_hash != start_node.self_hash){
        // std::cout<<"action: "<<hash_table[curr_hash].action<<std::endl;
        actions.push_front(hash_table[curr_hash].action);
        curr_hash = hash_table[curr_hash].parent_hash;
        // std::cout<<"parent hash: "<<curr_hash<<std::endl;
    }
    std::cout<<"Total steps: "<<actions.size()<<std::endl;

    std::cout<<"States Explored: "<<hash_table.size()<<std::endl;

    std::cout<<"Total Time: "<<(double)(clock() - start_time)/(double)CLOCKS_PER_SEC<<"s"<<std::endl;


    return actions;
}



int main(int argc, char* argv[])
{
    // DO NOT CHANGE THIS FUNCTION
    char* filename = (char*)("example.txt");
    if (argc > 1)
        filename = argv[1];

    cout << "Environment: " << filename << endl << endl;
    Env* env = create_env(filename);
    if (print_status)
    {
        cout << *env;
    }

    list<GroundedAction> actions = planner(env);

    cout << "\nPlan: " << endl;
    for (GroundedAction gac : actions)
    {
        cout << gac << endl;
    }

    return 0;
}

