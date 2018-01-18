class Sensor {
  public:
    Sensor(const int trig, const int echo);
    int get_distance();
    const int trig;
    const int echo;
};
