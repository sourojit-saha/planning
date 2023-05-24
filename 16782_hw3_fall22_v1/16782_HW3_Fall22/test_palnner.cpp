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
#include <queue>

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

    auto get_actions() const
    {
        return this->actions;
    }

    auto get_initial_conditions() const
    {
        return this->initial_conditions;
    }

    auto get_goal_conditions() const
    {
        return this->goal_conditions;
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
    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> gpreconditions;
    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> geffects;


public:
    GroundedAction(){}
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
            this->gpreconditions.insert(pc);
        }
        for (GroundedCondition pc : effects)
        {
            this->geffects.insert(pc);
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
        return this->gpreconditions;
    }

    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> get_effects() const
    {
        return this->geffects;
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

// TODOS:

// // Generate all possible combinations of symbols
// Ground all possible combinations of actions 
// Helper functions for A* search 
    // State class
    // Node Class 
    // Heuristic function

 // A* search

auto getAllCombinations(vector<string>symbols, int length, int index, int curr_len, vector<bool> check, vector<list<string>> &combinations){

    if(curr_len>length){
        return;
    }    
    if(curr_len==length){
        list<string> temp;
        for(int i=0;i<symbols.size();i++){
            if(check[i]){
                temp.push_back(symbols[i]);
            }
        }
        combinations.push_back(temp);
        return;
    }
    if(index==symbols.size()){
        return;
    }

    check[index]=true;
    getAllCombinations(symbols, length, index+1, curr_len+1, check, combinations);
    check[index]=false;
    getAllCombinations(symbols, length, index+1, curr_len, check, combinations);
}

auto getPermutations(vector<list<string>>combinations){
    vector<list<string>> permutations;
    for(auto combination:combinations){
        combination.sort();
        do{
            permutations.push_back(combination);
        }while(next_permutation(combination.begin(), combination.end()));
    }
    return permutations;
}

auto getGroundedActions(const unordered_set<Action, ActionHasher, ActionComparator> &actions, const unordered_set<string> &symbols_set){
    vector<string> symbols (symbols_set.begin(), symbols_set.end());
    vector<bool> check (symbols.size(), false);
    vector<list<string>> combinations;

    for(int i=1;i<=symbols.size();i++){
        getAllCombinations(symbols, i, 0, 0, check, combinations);
    }

    auto permutations = getPermutations(combinations);
    list<GroundedAction> possible_actions; 

    for(const auto &action: actions){
        for(const auto &permutation: permutations){
            if(action.get_args().size() == permutation.size()){
                                
                auto preconditions = action.get_preconditions();
                auto effects = action.get_effects();
                auto arg_names = action.get_args();

                unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> gpreconditions;
                unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> geffects;

                // String map from action_args to grounded_args
                unordered_map<string, string> string_map;
                for(auto& symbol: symbols_set){
                    string_map[symbol] = symbol;
                }

                auto it = permutation.begin();
                for(auto& arg: arg_names){
                    string_map[arg] = *it;
                    it++;
                }

                // ground precondition
                for(auto& condition: preconditions){
                    list<string> grounded_args;
                    for(auto& arg: condition.get_args()){
                        grounded_args.push_back(string_map[arg]);
                    }
                    GroundedCondition grounded_condition(condition.get_predicate(), grounded_args, condition.get_truth());
                    gpreconditions.insert(grounded_condition);
                }

                // ground effects
                for(auto& effect: effects){
                    list<string> grounded_args;
                    for(auto& arg: effect.get_args()){
                        grounded_args.push_back(string_map[arg]);
                    }
                    GroundedCondition grounded_effect(effect.get_predicate(), grounded_args, effect.get_truth());
                    geffects.insert(grounded_effect);
                }

                GroundedAction grounded_action(action.get_name(), permutation, gpreconditions, geffects);
                possible_actions.push_back(grounded_action);
            }
        }
    }
    return possible_actions;
}

class State{
    public:
        // constructors 
        State(){}
        State(const unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> &conditions){
            this->conditions = conditions;
        }

        // class members
        unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> conditions;
        
        // class methods
        auto set_conditions(const unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> &conditions){
            this->conditions = conditions;
        }

        string toString()  {
            string state_string ="";
            for(auto& condition: this->conditions){
                state_string += condition.toString();
            }
            return state_string;
        }

};

class StateHasher
{
    public:
    size_t operator()( State& state) 
    {
        return hash<string>{}(state.toString());
    }
};


class Node{
    public:

        Node(){
            this->g_value = std::numeric_limits<int>::max();
        }

        Node(const int &id, const State &state){
            this->id = id;
            this->state = state;
            this->g_value = std::numeric_limits<int>::max();
        }

        int id;
        size_t parent_hash;
        State state;
        int g_value;
        GroundedAction action; 
};


class NodeCostComparator{
    public:
        bool operator()(const Node &n1, const Node &n2){
            return n1.g_value > n2.g_value;
        }
};


class min_f_value{
    public:
        bool operator()(const pair<size_t, int>& p1, const pair<size_t, int>& p2) const {
            return p1.second > p2.second;
        }
};

auto h_value(const State &state, const State &goal){
    int h_value = goal.conditions.size();
    for(auto& condition: state.conditions){
        for(auto& goal_condition: goal.conditions){
            if(condition == goal_condition){
                h_value--;
            }
        }
    }
    return h_value;
}

auto getPath(Node goal_node, const size_t start_hash, unordered_map<size_t, Node> node_map, list<GroundedAction>& path){
    path.clear();
    auto current_node = goal_node;
    while(current_node.parent_hash != start_hash){
        path.push_front(current_node.action);
        current_node = node_map.at(current_node.parent_hash);
    }
    path.push_front(current_node.action);

}



list<GroundedAction> planner(Env* env){
    auto start_time = clock();
    list<GroundedAction> successor_actions = getGroundedActions(env->get_actions(), env->get_symbols());
    
    list<GroundedAction> path;

    State start_state = env->get_initial_conditions();
    State goal_state = env->get_goal_conditions();

    if(1){
        cout<<"Start State: "<<endl;
        for(auto& condition: start_state.conditions){
            cout<<condition<<" ";
        }
        cout<<endl;
        cout<<"Goal State: "<<endl;
        for(auto& condition: goal_state.conditions){
            cout<<condition<<" ";
        }
        cout<<endl;
    }

    priority_queue<pair<size_t, int>, vector<pair<size_t, int>>, min_f_value> open_list;
    unordered_set<size_t> closed_list;
    unordered_map<size_t, Node> node_map;

    // hash start state 
    size_t start_state_hash = StateHasher{}(start_state);

    Node start_node(start_state_hash, start_state);
    start_node.g_value = 0;
    node_map[start_state_hash] = start_node;
    open_list.push(make_pair(start_state_hash, start_node.g_value + h_value(start_state, goal_state)));

    // for(auto & action : successor_actions){

    //     auto preconditions = action.get_preconditions();
    //     auto effects = action.get_effects();

    //     std::cout<<"-------------actions"<<std::endl;
    //     std::cout<<action<<std::endl;

    //     std::cout<<"-------------preconditions"<<std::endl;
    //     for(auto & it : preconditions){
    //         std::cout<<it<<std::endl;
    //     }
    //     std::cout<<"-------------effects"<<std::endl;
    //     for(auto & it : effects){
    //         std::cout<<it<<std::endl;
    //     }
    // }


    while(!open_list.empty())
    // while(0)
    {
        auto current_node = open_list.top();
        open_list.pop();
        auto curr_state_hash = StateHasher{}(node_map[current_node.first].state);
        closed_list.insert(curr_state_hash);

        if(h_value(node_map[current_node.first].state, goal_state)==0){
            cout<<"Goal Found"<<endl;
            getPath(node_map[current_node.first], start_state_hash, node_map, path);
            break;
        }
        int counter = 0;

        for(auto& action: successor_actions){
            counter++;
            auto preconditions = action.get_preconditions();
            auto effects = action.get_effects();

            bool is_applicable = true;
            for(auto& condition: preconditions){
                if(node_map[current_node.first].state.conditions.find(condition) == node_map[current_node.first].state.conditions.end()){
                    is_applicable = false;
                    break;
                }
            }

            if(is_applicable){
                State new_state = node_map[current_node.first].state;
                for(auto effect: effects){
                    if(effect.get_truth()){
                        new_state.conditions.insert(effect);
                    }
                    else{
                        effect.set_truth(true);
                        new_state.conditions.erase(effect);
                    }
                }

                size_t new_state_hash = StateHasher{}(new_state);
                if(closed_list.find(new_state_hash)==closed_list.end()){
                    Node successor_node = node_map[new_state_hash];
                    int new_g_value = node_map[current_node.first].g_value + 1;
                    if(successor_node.g_value > new_g_value){
                        successor_node.g_value = new_g_value;
                        successor_node.parent_hash = current_node.first;
                        successor_node.action = action;
                        successor_node.state = new_state;
                        node_map[new_state_hash] = successor_node;
                        open_list.push(make_pair(new_state_hash, successor_node.g_value + h_value(new_state, goal_state)));
                    }
                }
                
            }
        }
    }
    
    cout<<"Time Taken: "<<(clock()-start_time)/double(CLOCKS_PER_SEC)<<endl;



    return path;
}


list<GroundedAction> given_planner(Env* env)
{
    // this is where you insert your planner

    // blocks world example
    list<GroundedAction> actions;
    actions.push_back(GroundedAction("MoveToTable", { "A", "B" }));
    actions.push_back(GroundedAction("Move", { "C", "Table", "A" }));
    actions.push_back(GroundedAction("Move", { "B", "Table", "C" }));

    return actions;
}

int main(int argc, char* argv[])
{
    // DO NOT CHANGE THIS FUNCTION
    char* filename = (char*)("Blocks.txt");
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
