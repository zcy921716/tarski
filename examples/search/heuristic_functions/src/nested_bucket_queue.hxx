#ifndef __NESTED_BUCKET_QUEUE__
#define __NESTED_BUCKET_QUEUE__

#include <cassert>
#include <iostream>
#include <queue>
#include <utility>
#include <vector>

namespace aptk {

namespace agnostic_int {

template<typename Value>
class Nested_Bucket_Queue  {
    static const int BUCKET_SIZE = 100;

    typedef std::tuple<int, int, Value> Entry;

    typedef std::vector<Value>  Bucket;
    typedef std::vector<Bucket> Bucket_Vec;
    std::vector<Bucket_Vec> buckets;
    mutable int current_bucket_no;
    mutable int current_bucket2_no;
    int num_entries;
    int num_pushes;

    void update_current_bucket_no() const {
        int num_buckets = buckets.size();
        while (current_bucket_no < num_buckets){
		int num_buckets2 = buckets[current_bucket_no].size();
		while (current_bucket2_no < num_buckets2 &&
		       buckets[current_bucket_no][current_bucket2_no].empty())
			++current_bucket2_no;

		if(current_bucket2_no >= num_buckets2){
			current_bucket2_no = 0;
			++current_bucket_no;
		}else
			break;
	}
    }

    void extract_sorted_entries(std::vector<Entry> &result) {
        // Generate vector with the entries of the queue in sorted
        // order, removing them from this queue as a side effect.
        assert(result.empty());
        result.reserve(num_entries);
        for (int key = current_bucket_no; num_entries != 0; ++key) {
            Bucket &bucket = buckets[key];
            for (size_t key2 = 0; key2 < bucket.size(); ++key2)
		    result.push_back(std::make_tuple(key,key2, bucket[key2]));
            num_entries -= bucket.size();
            Bucket empty_bucket;
            bucket.swap(empty_bucket);
        }
        current_bucket_no = 0;
        current_bucket2_no = 0;
    }
public:
    Nested_Bucket_Queue() : buckets(BUCKET_SIZE),current_bucket_no(0), current_bucket2_no(0), num_entries(0), num_pushes(0) {
	    for(int key = 0; key< buckets.size(); key++)
	    	    buckets[key].resize(1);
    }


    virtual ~Nested_Bucket_Queue() {
    }

    virtual void push(int key, int key2, const Value &value) {
        ++num_entries;
        ++num_pushes;
        assert(num_pushes > 0); // Check against overflow.
        int num_buckets = buckets.size();
        if (key >= num_buckets)
		buckets.resize( ( key * 2 ) + 1);
        else if (key < current_bucket_no)
            current_bucket_no = key;

        int num_buckets2 = buckets[key].size();
        if (key2 >= num_buckets2)
		buckets[key].resize((key2 * 2) + 1);
        else if (key2 < current_bucket2_no)
            current_bucket2_no = key2;
        buckets[key][key2].push_back(value);
    }

    virtual Value pop() {
        assert(num_entries > 0);
        --num_entries;
        update_current_bucket_no();
        Bucket &current_bucket = buckets[current_bucket_no][current_bucket2_no];
        Value top_element = current_bucket.back();
        current_bucket.pop_back();
        return top_element;
    }

    virtual bool empty() const {
        return num_entries == 0;
    }

    virtual void clear() {
        for (int i = current_bucket_no; num_entries != 0; ++i) {
		for (int j = current_bucket2_no; num_entries != 0; ++j) {
			int bucket_size = buckets[i][j].size();
			assert(bucket_size <= num_entries);
			num_entries -= bucket_size;
			buckets[i][j].clear();
		}
	}
        current_bucket_no = 0;
        current_bucket2_no = 0;
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
