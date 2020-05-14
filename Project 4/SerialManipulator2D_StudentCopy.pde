/**
 <h3>3DOF Serial Manipulator</h3>
 <p>Author: <a href="http://frank.dellaert.com">Frank Dellaert</a></p>
 <!--Creative Commons License--><a rel="license" href="http://creativecommons.org/licenses/by-nc-sa/2.5/">
 <img alt="Creative Commons License" style="border-width: 0" src="http://i.creativecommons.org/l/by-nc-sa/2.5/88x31.png"/></a><br/>
 This work is licensed under a <a rel="license" href="http://creativecommons.org/licenses/by-nc-sa/2.5/">Creative Commons Attribution-Noncommercial-Share Alike 2.5  License</a>.<!--/Creative Commons License--><!-- <rdf:RDF xmlns="http://web.resource.org/cc/" xmlns:dc="http://purl.org/dc/elements/1.1/" xmlns:rdf="http://www.w3.org/1999/02/22-rdf-syntax-ns#" xmlns:rdfs="http://www.w3.org/2000/01/rdf-schema#"><Work rdf:about=""><license rdf:resource="http://creativecommons.org/licenses/by-nc-sa/2.5/" /><dc:type rdf:resource="http://purl.org/dc/dcmitype/InteractiveResource" /></Work><License rdf:about="http://creativecommons.org/licenses/by-nc-sa/2.5/"><permits rdf:resource="http://web.resource.org/cc/Reproduction"/><permits rdf:resource="http://web.resource.org/cc/Distribution"/><requires rdf:resource="http://web.resource.org/cc/Notice"/><requires rdf:resource="http://web.resource.org/cc/Attribution"/><prohibits rdf:resource="http://web.resource.org/cc/CommercialUse"/><permits rdf:resource="http://web.resource.org/cc/DerivativeWorks"/><requires rdf:resource="http://web.resource.org/cc/ShareAlike"/></License></rdf:RDF>
 -->
 */

Pose2 base = new Pose2(0, 0, 0);
Pose2 tool = new Pose2(0, 0, 0);
Link2 [] links;
SerialLink2 manipulator;

double[] qd = new double[3]; // desired joint coordinates
double[] q = new double[3]; // actual joint coordinates
float pi = (float)Math.PI;
float xd=6, yd=5, theta_d=0; // desired tool frame
double Kp = 0.05; // Proportional controller

// demo kinematics: TF
// demo joint-space: FF
// demo jacobians: TT
// demo cartesian: FT
boolean SLIDERS = false; // Just use sliders
boolean CARTESIAN = true; // Do inverse Jacobian cartesian control

// Inverse Kinematics parameters
float L = 3.5, maxRange = 2*L, maxRange2 = maxRange*maxRange;
float L3 = 2, beta=theta_d;
float sign = -1; // control ellbow

// User interface stuff
HScrollbar[] scrollBar = new HScrollbar[3];  // scrollbars
PFont font; // font
int w = 800, h = 400;


void setup () {
  Tests.run(); // Run unit tests
  StudentTests.run(); // Run student tests

  size(800, 460); // create display space
  surface.setSize(800, 460); // needed on Mojave

  font = createFont("Helvetica", 12);
  textFont(font);
  clearTrail();
  frameRate(100);

  // Create scrollbars
  scrollBar[0] = new HScrollbar(0, h+48, w, 16, 1);
  scrollBar[1] = new HScrollbar(0, h+30, w, 16, 1);
  scrollBar[2] = new HScrollbar(0, h+12, w, 16, 1);
  q[0]=-0.1;
  q[1]=-0.1;
  q[2]=-0.1;

  // Create robot from Slides
  links = new Link2[3];
  links[0] = new Link2(3.5);
  links[1] = new Link2(3.5);
  links[2] = new Link2(2);
  manipulator = new SerialLink2(3, links, base, tool);
}

void draw() {
  background(255);

  // update and display the scrollbars
  for (int i=0;i<3;i++) {
    scrollBar[i].update();
    scrollBar[i].display();
  }

  if (mousePressed) {

    if (mouseY>400) {
      // Get joint angles from scrollbars
      for (int i=0;i<3;i++) {
        float x = scrollBar[i].getPos();
        qd[i] = pi*(2*x/w-1);
        if (SLIDERS) q[i] = qd[i];
      }
      Pose2 desired = manipulator.fkine(qd);
      xd = (float)desired.x();
      yd = (float)desired.y();
    }

    else {
      // Try inverse kinematics
      // Get desired tool position
      float xp = float(mouseX-w/3)/35.0;
      float yp = -float(mouseY-380)/35.0;
      // Calculate desired joint 3 position
      float x = xp - L3*cos(beta);
      float y = yp - L3*sin(beta);
      float r2 = x*x+y*y;
      if (r2<=maxRange2) {
        // set desired tool position
        xd=xp; yd=yp;
        // Do closed-form inverse kinematics as in PlanarIK.nb
        float theta2 = sign*2.0*atan(sqrt(maxRange2/r2-1));
        float theta1 =  atan2(y, x) - atan2(L*sin(theta2), L*(1+cos(theta2)));
        qd[0] = theta1;
        qd[1] = theta2;
        qd[2] = beta - theta1 - theta2;
      }
    }

    //clearTrail();
  } // mousePressed

  // set origin and scale
  translate(w/3, 380);
  scale(35);
  textSize(0.4);

  // draw workspace
  ellipseMode(CENTER);
  noFill();
  stroke(255, 240, 240);
  ellipse(2*cos(theta_d), 2*sin(theta_d), 14, 14);

  // Draw the trail
  Pose2 tool = manipulator.fkine(q);
  if (!SLIDERS) drawTrail(tool);

  if (CARTESIAN==false) {
    ///////////////////////////////////////////////////////////////////////
    ///NOTE: THIS IS WHERE THE STUDENT JOINT ANGLE TRAJECTORY CONTROL IS USED
    //STUDENTS DONT NEED TO WRITE CODE HERE THOUGH, UNLESS FOR TESTING
    ///////////////////////////////////////////////////////////////////////
    q = SerialLink2.proportionalJointAngleController(q, qd, Kp);
  }
  else {
    ///////////////////////////////////////////////////////////////////////
    ///NOTE: THIS IS WHERE THE STUDENT CARTESIAN TRAJECTORY CONTROL IS USED
    //STUDENTS DONT NEED TO WRITE CODE HERE THOUGH, UNLESS FOR TESTING
    ///////////////////////////////////////////////////////////////////////
    q = SerialLink2.proportionalCartesianController(xd, yd, theta_d, q, tool, Kp);
  }

  // Draw the robot
  Pose2 sTt = drawRobot(q, false, CARTESIAN);
}
