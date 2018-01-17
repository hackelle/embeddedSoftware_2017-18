class Drivetrain {
  public:
    Drivetrain(int le, int re, int lf, int lb, int rf, int rb);
    void Start();
    void Move(int left, int right);
    void Stop();

    int le;
    int lf;
    int lb;
    int re;
    int rf;
    int rb;
    int left;
    int right;
};
