#ifndef BIOT
#define BIOT
#include <iostream>
#include <chrono>
#include <memory>
#include <cassert>
namespace biot{
    template<typename T>
    class RingBuffer{
        private:
            std::unique_ptr<T[]> data_;
            std::size_t size_;
            std::size_t head_;
            std::size_t tail_;
            std::size_t capacity_;
        public:
            RingBuffer(std::size_t capacity) : data_(std::make_unique<T[]>(capacity)), size_(0), head_(0), tail_(0), capacity_(capacity)  {}
            ~RingBuffer() = default;
            T& operator[](std::size_t idx);
            const T& operator[](std::size_t idx) const;
            T& front();
            T& back();
            bool enqueue(const T& p);
            bool dequeue(T& p);
            bool is_full() const;
            bool is_empty() const;
            bool clear();
            std::size_t size() const;
            std::size_t capacity() const;
            std::size_t physical_index(std::size_t idx) const; //ring buffer opttimize trick if capacity == ^2 base 
    };  
    struct Fixed {
        Fixed(std::size_t count) : count(count) {}
        std::size_t count;
        template<typename T>
        bool ready(const RingBuffer<T>& buffer) const;
    };
    struct History {
        private:
            std::chrono::steady_clock::time_point start_;
            std::chrono::milliseconds duration_;
        public:
            explicit History(std::chrono::milliseconds ms) : start_(std::chrono::steady_clock::now()), duration_(ms) {}
            template<typename T>
            bool ready(const RingBuffer<T>& buffer);
            void reset();
    };
    template<typename T>
    class WindowView {
    private:
        const RingBuffer<T>* rb_;
        std::size_t first_;
        std::size_t count_;

    public:
        WindowView(RingBuffer<T>& rb,
                std::size_t first,
                std::size_t count) : rb_ (&rb), first_(first), count_(count) {}

        const T& operator[](size_t index) const;

        std::size_t size() const;
    };
    template<typename T, typename Policy>
    class Slidding_window{
        private:
            RingBuffer<T> window_buffer;
            Policy policy_;
        public:
            Slidding_window(std::size_t size, Policy policy) : window_buffer(size), policy_(policy) {} //copy policy obj to policy_ , update std::move(policy) for bigger policy feature
            std::size_t size();
            bool ready();
            bool clear();
            bool push(const T& p);
            WindowView<T> view();
    };
    template<typename T>
    inline bool Fixed::ready(const RingBuffer<T>& buffer)const
    {
        return buffer.size() >= count;
    }
    template<typename T>
    inline bool History::ready(const RingBuffer<T>& buffer)
    {
        (void)buffer;

        auto now = std::chrono::steady_clock::now();
        if( now-start_>= duration_){ //update start_ for a loop
            start_ = now;
            return true;
        };
        return false;
    }
    inline void History::reset(){
        start_ = std::chrono::steady_clock::now();
    }
    template<typename T>
    std::size_t RingBuffer<T>::physical_index(std::size_t idx) const{
        return (head_ + idx) & (capacity_ - 1);
    };
    template<typename T>
    T& RingBuffer<T>::operator[](std::size_t idx){
        assert(idx < size_ );
        std::size_t physical;
        if(capacity_ != 0 && capacity_ & (capacity_ -1) == 0){ //check for capacity_ == ^2
            return data_[physical_index(idx)];
        }
        else{
            physical = (head_ + idx)% capacity_;
        }
        return data_[physical];
    };
    template <typename T>
    const T& RingBuffer<T>::operator[](std::size_t idx) const{
        std::size_t physical = (head_+idx)% capacity_;
        return data_[physical];
    };
    template <typename T>
    T& RingBuffer<T>::front(){
        return data_[head_];
    }
    template <typename T>
    T& RingBuffer<T>::back(){
        return operator[](size_ - 1);
    }
    template <typename T>
    bool RingBuffer<T>::enqueue(const T& p){
        if (is_full()){
            return false;
        }
        data_[tail_] = p;
        tail_ = (tail_ + 1) % capacity_;
        size_++;
        return true;
    };
    template <typename T>
    bool RingBuffer<T>::dequeue(T& p){
        if (is_empty()){
            return false;
        }
        p = data_[head_];
        head_ = (head_ + 1) % capacity_;
        size_--;
        return true;
    };
    template<typename T>
    bool RingBuffer<T>::is_full() const{
        return size_ == capacity_;
    };
    template<typename T>
    bool RingBuffer<T>::is_empty() const{
        return size_ == 0;
    };
    template<typename T>
    bool RingBuffer<T>::clear(){
        head_ = 0;
        tail_ = 0;
        size_ = 0;
        return true;
    };
    template<typename T>
    std::size_t RingBuffer<T>::size() const{
        return size_;
    };
    template<typename T>
    std::size_t RingBuffer<T>::capacity() const{
        return capacity_;
    };
    template<typename T,typename Policy>
    std::size_t Slidding_window<T,Policy>::size(){
        return window_buffer.size();
    };
    template<typename T,typename Policy>
    bool Slidding_window<T,Policy>::ready(){
        return policy_.ready(window_buffer);
    };
    template<typename T,typename Policy>
    bool Slidding_window<T,Policy>::clear(){
        return window_buffer.clear();
    };
    template<typename T,typename Policy>
    bool Slidding_window<T,Policy>::push(const T& p){
        window_buffer.enqueue(p);
        return true;
    };
    template<typename T>
    const T& WindowView<T>::operator[](size_t index) const{
        return (*rb_)[index];
    };
    template<typename T>
    std::size_t WindowView<T>::size() const{
        return count_;
    };
    template<typename T,typename Policy>
    WindowView<T> Slidding_window<T,Policy>::view(){
        return WindowView<T>(window_buffer, 0, window_buffer.size());
    };
}
#endif 