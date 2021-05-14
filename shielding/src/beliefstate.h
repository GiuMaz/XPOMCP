#ifndef BELIEF_STATE_H
#define BELIEF_STATE_H

#include <vector>
#include <memory>

class STATE;
class SIMULATOR;

class BELIEF_META_INFO {
public:
    virtual void update(STATE *) {}
    virtual void clear() {}
    virtual std::unique_ptr<BELIEF_META_INFO> clone() const {
        return std::make_unique<BELIEF_META_INFO>(*this);
    }
};

class BELIEF_STATE
{
public:
    BELIEF_STATE();
    BELIEF_STATE(const BELIEF_META_INFO &meta);

    //BELIEF_STATE(const BELIEF_STATE &b) = default;

    // Free memory for all states
    void Free(const SIMULATOR& simulator);

    // Creates new state, now owned by caller
    STATE* CreateSample(const SIMULATOR& simulator) const;

    // Added state is owned by belief state
    void AddSample(STATE* state);

    // Make own copies of all samples
    void Copy(const BELIEF_STATE& beliefs, const SIMULATOR& simulator);

    // Move all samples into this belief state
    void Move(BELIEF_STATE& beliefs);

    bool Empty() const { return Samples.empty(); }
    int GetNumSamples() const { return Samples.size(); }
    const STATE* GetSample(int index) const { return Samples[index]; }

    BELIEF_META_INFO &get_metainfo() {
        return *metainfo;
    }

    const BELIEF_META_INFO &get_metainfo() const {
        return *metainfo;
    }

    void set_metainfo(const BELIEF_META_INFO &m) {
        metainfo = m.clone();
    }

private:

    std::vector<STATE*> Samples;
    std::unique_ptr<BELIEF_META_INFO> metainfo;
};

#endif // BELIEF_STATE_H
