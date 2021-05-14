#ifndef HISTORY_H
#define HISTORY_H

#include <vector>
#include <ostream>
#include <assert.h>

class HISTORY
{
public:

    struct ENTRY    // (action,observation)
    {
        ENTRY() { }

        ENTRY(int action, int obs)
        :   Action(action), Observation(obs)
        { }
        
        int Action;
        int Observation;
    };
    
    bool operator==(const HISTORY& history) const
    {
        if (history.History.size() != History.size())
            return false;
        for (int i = 0; i < History.size(); ++i)
            if (history.History[i].Action != History[i].Action
             || history.History[i].Observation != History[i].Observation)
                return false;
        return true;
    }
    
    void Add(int action, int obs = -1)  // add ENTRY(action, obs) to History
    { 
        History.push_back(ENTRY(action, obs));
    }
    
    void Pop()              // pop last ENTRY
    {
        History.pop_back();
    }
    
    void Truncate(int t)    // resizes the stack to dim t
    {
        History.resize(t);
    }
    
    void Clear()        // clears the History
    { 
        History.clear(); 
    }
    
    int Size() const    // history dim
    {
        return History.size();
    }
    
    ENTRY& operator[](int t)
    {
        assert(t >= 0 && t < History.size());
        return History[t];
    }

    const ENTRY& operator[](int t) const
    {
        assert(t >= 0 && t < History.size());
        return History[t];
    }

    ENTRY& Back()   // Returns a read/write reference to the data at the last element of the %vector.
    {
        assert(History.size() > 0);
        return History.back();
    }

    const ENTRY& Back() const
    {
        assert(History.size() > 0);
        return History.back();
    }

    void Display(std::ostream& ostr) const  // Shows the entire history (a1,o1),(a2,o2),...
    {
        for (int t = 0; t < History.size(); ++t)
        {
            ostr << "a=" << History[t].Action <<  " ";
            if (History[t].Observation >= 0)
                ostr << "o=" << History[t].Observation << " ";
        }
    }


private:

    std::vector<ENTRY> History; // Vector of ENTRIES
};

#endif // HISTORY
