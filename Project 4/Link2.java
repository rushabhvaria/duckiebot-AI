/// 2D Version, Modified DH convention, revolute joint
class Link2 {

  double offset_, a_;  

  Link2(double a) {
    a_=a;
  }

  // Transform
  public Pose2 T(double q) {
    double ct = Math.cos(q), st = Math.sin(q); 
    return new Pose2(a_*ct, a_*st, ct, st);
  }
}
