#ifndef PACKET
#define PACKET
#include <iostream>
#include <vector>
#include <span>
#include <cstring>
#include <chrono>
#include <queue>


namespace biot{
  enum flag_t : uint8_t{
    NONE = 0,
    CRASH_WARNING = 1 << 0,
    CRASH = 1 << 1
  };
  struct packet_t{
    uint32_t timestamp{};

    float accel_mag{};
    float roll{};
    float pitch{};
    float accel_jerk{};
    float velocity{};

    uint16_t seq{};
    uint8_t flag{}; 
    packet_t() = default;
    packet_t(float accel_mag,float roll,float pitch,float accel_jerk,float velocity,uint16_t seq, uint8_t flag) 
             : timestamp(0), 
          accel_mag(accel_mag),
          roll(roll),
          pitch(pitch),
          accel_jerk(accel_jerk),
          velocity(velocity),
          seq(seq),
          flag(flag)
    {}
    void set_flag(flag_t event, packet_t& p);
    void clear_flag(flag_t event, packet_t& p);
    bool is_valid(flag_t event, packet_t& p);
  };
  struct event_packet_t{
    uint8_t seq{};
    uint8_t flag{};
  };
  class BinarySerializer{
    public:
      static constexpr std::size_t packet_size = 5 * sizeof(float) + 1 * sizeof(uint8_t);
      packet_t deserialize(const uint8_t* data, std::size_t size);
      void serialize(const event_packet_t& p, std::queue<std::vector<uint8_t>>& queue);
  };
  class ByteWriter{
    private:
      std::vector<uint8_t> buffer;
    public:
      void write_float(float value);
      void write_uint8_t(uint8_t value);
      const std::vector<uint8_t>& data() const;
      void move_to(std::queue<std::vector<uint8_t>>& queue);
  };
  class ByteReader{
    private:
      const uint8_t* data_;
      std::size_t offset_ = 0;
      std::size_t size_;
    public:
      ByteReader(const uint8_t* data,std::size_t bytes): data_(data) , size_(bytes){}
      template <typename T>
      T read();
      void reset(const uint8_t* data, std::size_t size);
  };
  void packet_t::set_flag(flag_t event, packet_t& p){
    p.flag |= static_cast<uint8_t>(event);
  };
  void packet_t::clear_flag(flag_t event, packet_t& p){
    p.flag &= ~static_cast<uint8_t>(event);
  };
  bool packet_t::is_valid(flag_t event, packet_t& p){
    return (p.flag & static_cast<uint8_t>(event)) != 0;
  };
  void ByteWriter::move_to(std::queue<std::vector<uint8_t>>& queue){
      queue.push(std::move(buffer));
      buffer.clear();
  };
  void ByteWriter::write_uint8_t(uint8_t value){
    buffer.push_back(value);
  }
  const std::vector<uint8_t>& ByteWriter::data() const{
    return buffer;
  }
  template<typename T>
  T ByteReader::read(){
    T data;
    memcpy(&data, data_ + offset_, sizeof(T));
    offset_ += sizeof(T);
    return data;
  }
  /*uint8_t ByteReader::read_uint8_t(){
    return buffer[offset++];
  } for reference */ 
  packet_t BinarySerializer::deserialize(const uint8_t* data, std::size_t size){
    ByteReader reader(data,size);
    packet_t p;
    //update timestamp every newly deserialize packet
    p.timestamp =
    static_cast<uint32_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()
        ).count()
    ); //cast base uint32_t static by ms of the time happen since last epoch and extract value via count()
    p.accel_mag = reader.read<float>();
    p.accel_jerk = reader.read<float>();
    p.roll = reader.read<float>();
    p.pitch = reader.read<float>();
    p.velocity = reader.read<float>();
    p.seq = reader.read<uint16_t>();
    p.flag = reader.read<uint8_t>();

    return p;
  }

  void BinarySerializer::serialize(const event_packet_t& p, std::queue<std::vector<uint8_t>>& queue){
    ByteWriter writer;
    writer.write_uint8_t(p.seq);
    writer.write_uint8_t(p.flag);

    writer.move_to(queue);
  };
  void ByteReader::reset(const uint8_t* data, std::size_t size){
    data_ = data;
    size_ = size;
    offset_ =0;
  }
}
#endif