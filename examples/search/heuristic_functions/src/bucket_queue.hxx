#ifndef __BUCKET_QUEUE__
#define __BUCKET_QUEUE__

#include <cassert>
#include <iostream>
#include <queue>
#include <utility>
#include <vector>

namespace aptk {

namespace agnostic_int {

template<typename Value>
class BucketQueue  {
    static const int BUCKET_SIZE = 100;

    typedef std::pair<int, Value> Entry;

    typedef std::vector<Value> Bucket;
    std::vector<Bucket> buckets;
    mutable int current_bucket_no;
    int num_entries;
    int num_pushes;

    void update_current_bucket_no() const {
        int num_buckets = buckets.size();
        while (current_bucket_no < num_buckets &&
               buckets[current_bucket_no].empty())
            ++current_bucket_no;
    }

    void extract_sorted_entries(std::vector<Entry> &result) {
        // Generate vector with the entries of the queue in sorted
        // order, removing them from this queue as a side effect.
        assert(result.empty());
        result.reserve(num_entries);
        for (int key = current_bucket_no; num_entries != 0; ++key) {
            Bucket &bucket = buckets[key];
            for (size_t i = 0; i < bucket.size(); ++i)
                result.push_back(std::make_pair(key, bucket[i]));
            num_entries -= bucket.size();
            Bucket empty_bucket;
            bucket.swap(empty_bucket);
        }
        current_bucket_no = 0;
    }
public:
    BucketQueue() : buckets(BUCKET_SIZE),current_bucket_no(0), num_entries(0), num_pushes(0) {

    }


    virtual ~BucketQueue() {
    }

    virtual void push(int key, const Value &value) {
        ++num_entries;
        ++num_pushes;
        assert(num_pushes > 0); // Check against overflow.
        int num_buckets = buckets.size();
        if (key >= num_buckets)
		buckets.resize( ( key * 2 ) + 1);
		//	    buckets.resize(num_buckets * 2);
//            buckets.resize(key + 1);
        else if (key < current_bucket_no)
            current_bucket_no = key;
        buckets[key].push_back(value);
    }

    virtual Entry pop() {
        assert(num_entries > 0);
        --num_entries;
        update_current_bucket_no();
        Bucket &current_bucket = buckets[current_bucket_no];
        Value top_element = current_bucket.back();
        current_bucket.pop_back();
        return std::make_pair(current_bucket_no, top_element);
    }

    virtual bool empty() const {
        return num_entries == 0;
    }

    virtual void clear() {
        for (int i = current_bucket_no; num_entries != 0; ++i) {
            // assert(Utils::in_bounds(i, buckets));
            int bucket_size = buckets[i].size();
            assert(bucket_size <= num_entries);
            num_entries -= bucket_size;
            buckets[i].clear();
        }
        current_bucket_no = 0;
        assert(num_entries == 0);
        num_pushes = 0;
    }


    virtual void add_virtual_pushes(int num_extra_pushes) {
        num_pushes += num_extra_pushes;
    }
};


}

}

#endif
