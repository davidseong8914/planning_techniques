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

//I added
#include <queue>
#include <vector>
#include <functional>
#include <chrono>

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

/*
Run command:
g++ -std=c++17 planner.cpp -o planner.out
./planner.out example.txt
*/

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

    // I added
    const unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator>& 
    get_initial_conditions() const {
        return this->initial_conditions;
    }

    const unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator>& 
    get_goal_conditions() const {
        return this->goal_conditions;
    }

    const unordered_set<Action, ActionHasher, ActionComparator>& 
    get_actions() const {
        return this->actions;
}
};

class GroundedAction
{
private:
    string name;
    list<string> arg_values;

public:
    GroundedAction(string name, list<string> arg_values)
    {
        this->name = name;
        for (string ar : arg_values)
        {
            this->arg_values.push_back(ar);
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


//////////////////////
/// Implementation ///
//////////////////////


// State definition
/// quick look up
typedef unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> State;

// State hashing and comparison functions
struct StateHasher {
    size_t operator()(const State& state) const {
        size_t hash_val = 0;
        for (const auto& cond : state) {
            hash_val ^= GroundedConditionHasher()(cond);
        }
        return hash_val;
    }
};

struct StateComparator {
    bool operator()(const State& lhs, const State& rhs) const {
        if (lhs.size() != rhs.size()) return false;
        for (const auto& cond : lhs) {
            if (rhs.find(cond) == rhs.end()) return false;
        }
        return true;
    }
};

// State Node
struct Node {
    State state;
    list<GroundedAction> actions_so_far;
    int g_cost; // cost
    int h_cost; // heuristic
    int f_cost; // g_cost + h_cost
    
    // Used for priority queue comparison
    bool operator>(const Node& other) const {
        return f_cost > other.f_cost;
    }
};

list<GroundedAction> planner(Env* env) {

    // Initial State
    State initial_state;
    for (const auto& condition : env->get_initial_conditions()) {
        initial_state.insert(condition);
    }
    
    // Goal State
    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> goal_conditions;
    for (const auto& goal : env->get_goal_conditions()) {
        goal_conditions.insert(goal);
    }
    
    // Check if state satisfies goal conditions
    auto is_goal_state = [&goal_conditions](const State& state) {
        for (const auto& goal : goal_conditions) {
            if (state.find(goal) == state.end()) {
                return false;
            }
        }
        return true;
    };
    
    // Heuristic function: count unsatisfied goals
    auto calculate_heuristic = [&goal_conditions](const State& state) {
        int unsatisfied = 0;
        for (const auto& goal : goal_conditions) {
            if (state.find(goal) == state.end()) {
                unsatisfied++;
            }
        }
        return unsatisfied;
    };
    
    // Generate ground actions from a parametrized action
    auto ground_action = [&env](const Action& action, const vector<string>& values, const vector<int>& indices) {
        list<string> arg_values;
        auto args = action.get_args();
        auto args_it = args.begin();
        
        unordered_map<string, string> param_map;
        for (size_t i = 0; i < indices.size(); i++) {
            param_map[*args_it] = values[indices[i]];
            args_it++;
        }
        
        for (const auto& arg : args) {
            arg_values.push_back(param_map[arg]);
        }
        
        return GroundedAction(action.get_name(), arg_values);
    };
    
    // Ground a condition using parameter mapping
    auto ground_condition = [](const Condition& cond, const unordered_map<string, string>& param_map) {
        list<string> arg_values;
        for (const auto& arg : cond.get_args()) {
            if (param_map.find(arg) != param_map.end()) {
                arg_values.push_back(param_map.at(arg));
            } else {
                arg_values.push_back(arg);
            }
        }
        return GroundedCondition(cond.get_predicate(), arg_values, cond.get_truth());
    };
    
    // Generate all possible ground actions
    auto get_applicable_actions = [&env, &ground_action, &ground_condition](const State& state) {
        vector<GroundedAction> applicable_actions;
        vector<string> symbols(env->get_symbols().begin(), env->get_symbols().end());
        
        for (const Action& action : env->get_actions()) {
            auto args = action.get_args();
            int num_args = args.size();
            
            vector<vector<int>> indices_list;
            function<void(vector<int>&, int)> generate_indices = [&](vector<int>& indices, int arg_index) {
                if (arg_index == num_args) {
                    indices_list.push_back(indices);
                    return;
                }
                for (int i = 0; i < symbols.size(); i++) {
                    indices.push_back(i);
                    generate_indices(indices, arg_index + 1);
                    indices.pop_back();
                }
            };
            
            vector<int> indices;
            generate_indices(indices, 0);
            
            // For each possible ground action
            for (const auto& indices : indices_list) {
                unordered_map<string, string> param_map;
                auto args_it = args.begin();
                for (int i = 0; i < indices.size(); i++) {
                    param_map[*args_it] = symbols[indices[i]];
                    args_it++;
                }
                
                // Check if preconditions are satisfied
                bool applicable = true;
                for (const auto& precond : action.get_preconditions()) {
                    GroundedCondition gc = ground_condition(precond, param_map);
                    if (gc.get_truth()) {
                        if (state.find(gc) == state.end()) {
                            applicable = false;
                            break;
                        }
                    } else {
                        GroundedCondition positive_gc(gc.get_predicate(), gc.get_arg_values(), true);
                        if (state.find(positive_gc) != state.end()) {
                            applicable = false;
                            break;
                        }
                    }
                }
                
                if (applicable) {
                    applicable_actions.push_back(ground_action(action, symbols, indices));
                }
            }
        }
        
        return applicable_actions;
    };
    
    // Apply action 
    auto apply_action = [&env, &ground_condition](const State& state, const GroundedAction& gaction) {
        State new_state = state;
        Action action = env->get_action(gaction.get_name());
        
        unordered_map<string, string> param_map;
        auto args = action.get_args();
        auto args_it = args.begin();
        auto values = gaction.get_arg_values();
        auto values_it = values.begin();
        
        while (args_it != args.end() && values_it != values.end()) {
            param_map[*args_it] = *values_it;
            args_it++;
            values_it++;
        }
        
        // Apply effects
        for (const auto& effect : action.get_effects()) {
            GroundedCondition gc = ground_condition(effect, param_map);
            if (gc.get_truth()) {
                new_state.insert(gc);
            } else {
                GroundedCondition positive_gc(gc.get_predicate(), gc.get_arg_values(), true);
                new_state.erase(positive_gc);
            }
        }
        
        return new_state;
    };


    ////////////////// PLANNING //////////////////

    // A* 
    priority_queue<Node, vector<Node>, greater<Node>> open_list;
    
    // Visited
    unordered_set<State, StateHasher, StateComparator> closed_set;
    
    // initialize initial node
    Node initial_node;
    initial_node.state = initial_state;
    initial_node.actions_so_far = {};
    initial_node.g_cost = 0;
    initial_node.h_cost = calculate_heuristic(initial_state);
    initial_node.f_cost = initial_node.g_cost + initial_node.h_cost;
    
    // add initial node
    open_list.push(initial_node);
    
    // explanded states
    int states_expanded = 0;
    // time
    auto start_time = chrono::high_resolution_clock::now();
    
    // A* 
    while (!open_list.empty()) {
        // get lowest f option
        Node current = open_list.top();
        open_list.pop();
        
        // check is visited
        if (closed_set.find(current.state) != closed_set.end()) {
            continue;
        }
        
        // add to closed
        closed_set.insert(current.state);
        states_expanded++;
        
        // check for goal
        if (is_goal_state(current.state)) {
            auto end_time = chrono::high_resolution_clock::now();
            auto duration = chrono::duration_cast<chrono::milliseconds>(end_time - start_time).count();
            
            if (print_status) {
                cout << "Goal reached!" << endl;
                cout << "States expanded: " << states_expanded << endl;
                cout << "Plan length: " << current.actions_so_far.size() << endl;
                cout << "Time taken: " << duration << " ms" << endl;
            }
            
            return current.actions_so_far;
        }

        // query applicable actions
        vector<GroundedAction> applicable_actions = get_applicable_actions(current.state);
        
        // iterate applicable actions
        for (const auto& action : applicable_actions) {
            State new_state = apply_action(current.state, action);
            
            // skip visited
            if (closed_set.find(new_state) != closed_set.end()) {
                continue;
            }
            
            // add to open list
            Node new_node;
            new_node.state = new_state;
            new_node.actions_so_far = current.actions_so_far;
            new_node.actions_so_far.push_back(action);

            new_node.g_cost = current.g_cost + 1;  // action cost + 1
            new_node.h_cost = calculate_heuristic(new_state);
            new_node.f_cost = new_node.g_cost + new_node.h_cost;
            
            open_list.push(new_node);
        }
    }

    // failed
    cout << "Failed: Goal not reached";
    return {};
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