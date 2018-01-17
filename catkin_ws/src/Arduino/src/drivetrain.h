class Drivetrain {
  public:
    Drivetrain(const int le, const int re, const int lf, const int lb, const int rf, const int rb);
    void Start();
    void Move(int left, int right);
    void Stop();

    const int le;
    const int lf;
    const int lb;
    const int re;
    const int rf;
    const int rb;
    int left;
    int right;
};
