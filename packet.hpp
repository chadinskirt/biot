#ifndef PACKET
#define PACKET
#include <iostream>
#include <vector>
#include <span>
#include <cstring>
#include <chrono>


namespace biot{
  enum flag_t : uint8_t{
    NONE = 0,
    CRASH_WARNING = 1 << 0,
    CRASH = 1 << 1
  };
  struct packet_t{
    uint32_t timestamp;

    float accel_mag;
    float roll;
    float pitch;
    float accel_jerk;
    float velocity;

    uint8_t flag;
    packet_t() : timestamp(0), accel_mag(0.0f), roll(0.0f), pitch(0.0f), velocity(0), flag(0){} 
    static void set_flag(packet_t& p, flag_t event);
    static void clear_flag(packet_t& p, flag_t event);
    static bool is_valid(packet_t&p, flag_t event);
  };
  class BinarySerializer{
    public:
      static constexpr std::size_t packet_size = 5 * sizeof(float) + 1 * sizeof(uint8_t);
      packet_t deserialize(const uint8_t* data, std::size_t size);
      std::vector<uint8_t> serialize(const packet_t& p);
  };
  class ByteWriter{
    private:
      std::vector<uint8_t> buffer;
    public:
      void write_float(float value);
      void write_uint8_t(uint8_t value);
      const std::vector<uint8_t>& data() const;
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
  static void set_flag(packet_t& p, flag_t event){
    p.flag |= static_cast<uint8_t>(event);
  };
  static void clear_flag(packet_t& p, flag_t event){
    p.flag &= ~static_cast<uint8_t>(event);
  };
  static bool is_valid(packet_t&p, flag_t event){
    return (p.flag & static_cast<uint8_t>(event)) != 0;
  };
  void ByteWriter::write_float(float value){
    uint8_t* ptr = reinterpret_cast<uint8_t*>(&value);
    buffer.insert(buffer.end(), ptr, ptr + sizeof(float));
  }
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
    p.roll = reader.read<float>();
    p.pitch = reader.read<float>();
    p.accel_jerk = reader.read<float>();
    p.velocity = reader.read<float>();
    p.flag = reader.read<uint8_t>();

    return p;
  }

  std::vector<uint8_t> BinarySerializer::serialize(const packet_t& p){
    ByteWriter writer;
    writer.write_float(p.accel_mag);
    writer.write_float(p.roll);
    writer.write_float(p.pitch);
    writer.write_float(p.accel_jerk);
    writer.write_float(p.velocity);
    writer.write_uint8_t(p.flag);

    return writer.data();
  };
  void ByteReader::reset(const uint8_t* data, std::size_t size){
    data_ = data;
    size_ = size;
    offset_ =0;
  }
}
#endif