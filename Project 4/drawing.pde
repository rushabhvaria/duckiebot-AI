/// Apply a 2D rigid transformation to Processing Applet
void apply(Pose2 p) {
  // Note the negative signs here is to put Y-axis up again
  translate((float)p.x(), -(float)p.y());
  rotate(-(float)p.theta());
}

// Draw a rotational joint
void drawJoint(float x, boolean textual, double q) {
  stroke(0);
  strokeWeight(0.03);

  // draw origin
  ellipseMode(CENTER);
  fill(255, 0, 0);
  ellipse(x, 0, 0.2, 0.2);
  
  if (textual) {
    pushMatrix();
    stroke(0);
    fill(150, 150, 150);
    rotate(-(float)q);
    while (q>PI) q-=2*PI;
    while (q<-PI) q+=2*PI;
    text(String.format("%-3.0f", (float)(180*q/Math.PI)), 0, -0+1);
    popMatrix();
  }  
}

void drawJoint(float x) {
  drawJoint(x, false, 0);
}

// Draw a link along the x-axis, with width w
void drawLink(float x0, float x1, float w) {
  stroke(0);
  strokeWeight(0.03);

  rectMode(CORNERS);
  noFill();
  float d=w/2;
  rect(x0-d, -d, x1+d, d, 5);
  strokeWeight(0.01);
  line(x0, 0, x1, 0);
  strokeWeight(0.02);
  stroke(255, 0, 0);
  line(0, 0, 1, 0);
  stroke(0, 255, 0);
  line(0, 0, 0, -1);
}

/// Draw a frame in processing, with textual option
void drawFrame(Pose2 p, boolean textual) {
  pushMatrix();
  apply(p);
  strokeWeight(0.05);
  stroke(255, 0, 0);
  line(0, 0, 1, 0);
  stroke(0, 255, 0);
  line(0, 0, 0, -1);
  // dimension
  if (textual) {
    stroke(0);
    rotate((float)p.theta());
    float x = (float)p.x();
    float y = (float)p.y();
    text(String.format("%-3.1f, %-3.1f, %-4.1f", p.x(), p.y(), p.theta()*180/Math.PI), 0, -0+1);
  }  
  popMatrix();
}

/// Draw a frame in processing
void drawFrame(Pose2 p) {
  drawFrame(p, false);
}

// Draw a trail
int trailLength = 60;
float[] trail_x = new float[trailLength], trail_y = new float[trailLength];
void clearTrail() {
  for (int i=0;i<trailLength;i++) {
    trail_x[i]=0;
    trail_y[i]=-100;
  }
}
void drawTrail(Pose2 tool) {
  ellipseMode(CENTER);
  noStroke();
  fill(200, 200, 250);
  // if no evolution, just draw
  if (abs(trail_x[trailLength-1]-(float)tool.x())<0.1 && abs(trail_y[trailLength-1]-(float)tool.y())<0.1)
    for (int i=0;i<trailLength;i++) {
      ellipse(trail_x[i], -trail_y[i], 0.2, 0.2);
    }
  else {  
    // otherwise advance trail
    for (int i=0;i<trailLength-1;i++) {
      trail_x[i]=trail_x[i+1];
      trail_y[i]=trail_y[i+1];
      ellipse(trail_x[i], -trail_y[i], 0.2, 0.2);
    }
    trail_x[trailLength-1] = (float)tool.x();
    trail_y[trailLength-1] = (float)tool.y();
  }
}

/// Draw an arrow, 
// adapted from http://processing.org/discourse/beta/num_1219607845.html
// does y inversion internally !
void arrow(float x1, float y1, float dx, float dy, float size) {
  line(x1, -y1, x1+dx, -y1-dy);
  pushMatrix();
  translate(x1+dx, -y1-dy);
  float a = atan2(dx, dy);
  rotate(a);
  line(0, 0, size, size);
  line(0, 0, -size, size);
  popMatrix();
} 

// Draw robot and return Tool pose, with optional dimensions flag
Pose2 drawRobot(double q[], boolean dimensions, boolean jacobian) {
  drawFrame(new Pose2(0, 0, 0));

  // draw all links, note that transformations in drawLink are cumulative!
  pushMatrix(); // save current drawing transformation
  apply(base);
  for (int i=0;i<3;i++) {
    Link2 link = links[i];
    drawJoint(0, true, q[i]);
    apply(link.T(q[i]));
    drawLink(-(float)link.a_, 0, 1);
    drawFrame(new Pose2(0, 0, 0));
  }
  popMatrix(); // recover global drawing transformation

  // Check forward kinematics by drawing tool frame
  Pose2 sTt = manipulator.fkine(q);
  boolean textual = true;
  drawFrame(sTt, textual);

  // Possibly draw Jacobian
  if (jacobian) {
    // Calculate analytical Jacobian for three-link manipulator
    double[][] J = manipulator.jacobian3(q);
    strokeWeight(0.05);
    float x = (float)sTt.x(), y = (float)sTt.y();
    stroke(255, 0, 0);
    arrow(x, y, (float)J[0][0], (float)J[1][0], 0.2);
    stroke(0, 255, 0);
    arrow(x, y, (float)J[0][1], (float)J[1][1], 0.2);
    stroke(0, 255, 255);
    arrow(x, y, (float)J[0][2], (float)J[1][2], 0.2);
  }

  return sTt;
}

// Draw robot and return Tool pose
Pose2 drawRobot(double q[]) {
  return drawRobot(q, false, false);
}
