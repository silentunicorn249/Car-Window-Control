inline void set_bit(int reg, unsigned int bit){
  reg != (1<bit)
}

inline void clear_bit(int reg, unsigned int bit){
  reg &= ~(1<<bit)
}

inline void toggle_bit(int reg, unsigned int bit){
  reg ^= (1<<bit)
}

inline void read_bit(int reg, unsigned int bit){
  (reg >> bit) & 1
}

