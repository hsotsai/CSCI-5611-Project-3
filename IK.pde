//Inverse Kinematics
//CSCI 5611 IK [Solution]
// Stephen J. Guy <sjguy@umn.edu>


void setup(){
  size(960,640);
  surface.setTitle("Inverse Kinematics [CSCI 5611 Example]");
  strokeWeight(2);

  pos = new Vec2(random(840)+60,100);
  vel = new Vec2(random(500)+50,random(200)+50);

  fk();
  leg_fk();
  solve();
  solve_walk();
  startTime = millis() / 1000.0;
  walkTime = startTime;
  
}

class hitInfo{
  public boolean hit = false;
  public float t = 9999999.99;
}

hitInfo lineCircleIntesect(Vec2 center, float r, Vec2 l_start, Vec2 l_dir, float l_len){
  hitInfo hit = new hitInfo();
  
  //Step 2: Compute W - a displacement vector pointing from the start of the line segment to the center of the circle
    Vec2 toCircle = center.minus(l_start);
    
    //Step 3: Solve quadratic equation for intersection point (in terms of l_dir and toCircle)
    float a = 1;  //Lenght of l_dir (we noramlized it)
    float b = -2*dot(l_dir,toCircle); //-2*dot(l_dir,toCircle)
    float c = toCircle.lengthSqr() - (r+5)*(r+5); //different of squared distances
    
    float d = b*b - 4*a*c; //discriminant 
    
    if (d >=0 ){ 
      //If d is positive we know the line is colliding, but we need to check if the collision line within the line segment
      //  ... this means t will be between 0 and the lenth of the line segment
      float t1 = (-b - sqrt(d))/(2*a); //Optimization: we only take the first collision [is this safe?]
      //println(hit.t,t1,t2);
      if (t1 > 0 && t1 < l_len){
        hit.hit = true;
        hit.t = t1;
      } 
    }
    
  return hit;
}

int r = 5;
float rad = 50;
float floor = 640;
float speed=2.5;
float step = 5;
int dex=0;
float startTime;
float walkTime;
float armW = 40;
float clawW = 5;
float legW = 50;
float lbody = 100;
float wbody = 250;
float startX = 300;
float startY = 475;
Vec2 pos;
Vec2 vel;
Vec2 acc;
Vec2 gravity = new Vec2(0,800);

//Root
Vec2 body = new Vec2(startX,startY);

Vec2 root = new Vec2(startX,startY);
Vec2 alt_root = new Vec2(startX+wbody,startY);
Vec2 leg_root = new Vec2(startX,startY+lbody/2);
Vec2 alt_leg_root = new Vec2(startX+wbody,startY+lbody/2);

Vec2 goal = new Vec2(startX,startY-200);
Vec2 alt_goal = new Vec2(startX+wbody,startY-200);

Vec2 leg_goal = new Vec2(leg_root.x,640);
Vec2 alt_leg_goal = new Vec2(alt_leg_root.x,640);

//Upper Arm
float l0 = 90; 
float a0 = 0.3; //Shoulder joint

//Lower Arm
float l1 = 80;
float a1 = 0.3; //Elbow joint

//Hand
float l2 = 50;
float a2 = 0.3; //Wrist joint

//Hand
float l2_claw = 50;
float a2_claw = 0.3; //Wrist joint

//Claw
float l3 = 80;
float a3 = 0.3; //Wrist joint

//Claw
float l3_claw = 80;
float a3_claw = 0.3; //Wrist joint

Vec2 start_l1,start_l2,start_l2_claw,start_l3,start_l3_claw,endPoint,endPoint_claw;

                                    //Upper Arm
                                    float alt_l0 = 90; 
                                    float alt_a0 = 0.3; //Shoulder joint
                                    
                                    //Lower Arm
                                    float alt_l1 = 80;
                                    float alt_a1 = 0.3; //Elbow joint
                                    
                                    //Hand
                                    float alt_l2 = 50;
                                    float alt_a2 = 0.3; //Wrist joint
                                    
                                    //Hand
                                    float alt_l2_claw = 50;
                                    float alt_a2_claw = 0.3; //Wrist joint
                                    
                                    //Claw
                                    float alt_l3 = 80;
                                    float alt_a3 = 0.3; //Wrist joint
                                    
                                    //Claw
                                    float alt_l3_claw = 80;
                                    float alt_a3_claw = 0.3; //Wrist joint
                                    
                                    Vec2 alt_start_l1,alt_start_l2,alt_start_l2_claw,alt_start_l3,alt_start_l3_claw,alt_endPoint,alt_endPoint_claw;

                                                                                      
                                                                                      //Thigh Arm
                                                                                      float leg_l0 = 90; 
                                                                                      float leg_a0 = 0.3; //Hip joint
                                                                                      
                                                                                      //Calf Arm
                                                                                      float leg_l1 = 80;
                                                                                      float leg_a1 = 0.3; //Knee joint
                                                                                      
                                                                                      Vec2 leg_start_l1,leg_endPoint;
                                                                                      
                                                                                      //Thigh Arm
                                                                                      float alt_leg_l0 = 90; 
                                                                                      float alt_leg_a0 = 0.3; //Hip joint
                                                                                      
                                                                                      //Calf Arm
                                                                                      float alt_leg_l1 = 80;
                                                                                      float alt_leg_a1 = 0.3; //Knee joint
                                                                                      
                                                                                      Vec2 alt_leg_start_l1,alt_leg_endPoint;
                                                                                      
                                                                                      //Thigh Arm
                                                                                      float lleg_l0 = 110; 
                                                                                      float lleg_a0 = 0.3; //Hip joint
                                                                                      
                                                                                      //Calf Arm
                                                                                      float lleg_l1 = 100;
                                                                                      float lleg_a1 = 0.3; //Knee joint
                                                                                      
                                                                                      Vec2 lleg_start_l1,lleg_endPoint;
                                                                                      
                                                                                      //Thigh Arm
                                                                                      float lalt_leg_l0 = 110; 
                                                                                      float lalt_leg_a0 = 0.3; //Hip joint
                                                                                      
                                                                                      //Calf Arm
                                                                                      float lalt_leg_l1 = 100;
                                                                                      float lalt_leg_a1 = 0.3; //Knee joint
                                                                                      
                                                                                      Vec2 lalt_leg_start_l1,lalt_leg_endPoint;
                                                                                      
                                                                                      //Thigh Arm
                                                                                      float aleg_l0 = 70; 
                                                                                      float aleg_a0 = 0.3; //Hip joint
                                                                                      
                                                                                      //Calf Arm
                                                                                      float aleg_l1 = 60;
                                                                                      float aleg_a1 = 0.3; //Knee joint
                                                                                      
                                                                                      Vec2 aleg_start_l1,aleg_endPoint;
                                                                                      
                                                                                      //Thigh Arm
                                                                                      float aalt_leg_l0 = 70; 
                                                                                      float aalt_leg_a0 = 0.3; //Hip joint
                                                                                      
                                                                                      //Calf Arm
                                                                                      float aalt_leg_l1 = 60;
                                                                                      float aalt_leg_a1 = 0.3; //Knee joint
                                                                                      
                                                                                      Vec2 aalt_leg_start_l1,aalt_leg_endPoint;

void solve(){

  if( fToggle ) goal = new Vec2(mouseX, mouseY);
  if( gToggle ) alt_goal = new Vec2(mouseX, mouseY);
  
  Vec2 startToGoal, startToEndEffector;
  float dotProd, angleDiff;
  
  a3_claw = 1.5708;
  fk();
  a3 = -1.5708;
  fk();
  ///a2_claw = -1.5708;
  //fk();
  //a2 = 1.5708;
  //fk();
  
  //Update elbow joint
  startToGoal = goal.minus(start_l2_claw);
  startToEndEffector = endPoint_claw.minus(start_l2_claw);
  dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
  dotProd = clamp(dotProd,-1,1);
  angleDiff = acos(dotProd);
  if (cross(startToGoal,startToEndEffector) < 0){
    a2_claw += angleDiff;
    a2_claw = min(a2_claw, -1.2708);
  }else{
    a2_claw -= angleDiff;
    a2_claw = max(a2_claw, -1.5708);
  }
  fk(); //Update link positions with fk (e.g. end effector changed)
  
  //Update elbow joint
  startToGoal = goal.minus(start_l2);
  startToEndEffector = endPoint.minus(start_l2);
  dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
  dotProd = clamp(dotProd,-1,1);
  angleDiff = acos(dotProd);
  if (cross(startToGoal,startToEndEffector) < 0){
    a2 += angleDiff;
    a2 = min(a2, 1.5708);
  }else{
    a2 -= angleDiff;
    a2 = max(a2, 1.2708);
  }
  fk(); //Update link positions with fk (e.g. end effector changed)
  
  
  //Update elbow joint
  startToGoal = goal.minus(start_l1);
  startToEndEffector = endPoint.minus(start_l1);
  dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
  dotProd = clamp(dotProd,-1,1);
  angleDiff = acos(dotProd);
  if (cross(startToGoal,startToEndEffector) < 0){
    a1 += angleDiff;
    a1 = min(a1, 1.5708);
  }else{
    a1 -= angleDiff;
    a1 = max(a1, 0);
  }
  fk(); //Update link positions with fk (e.g. end effector changed)
  
  //Update shoulder joint
  startToGoal = goal.minus(root);
  if (startToGoal.length() < .0001) return;
  startToEndEffector = endPoint.minus(root);
  dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
  dotProd = clamp(dotProd,-1,1);
  angleDiff = acos(dotProd);
  if (cross(startToGoal,startToEndEffector) < 0){
    a0 += angleDiff;
    a0 = min(a0, -1.3708);
  }else{
    a0 -= angleDiff;
    a0 = max(a0, -3.5587375);
  }
  fk(); //Update link positions with fk (e.g. end effector changed)
  
  


                                                                            alt_a3_claw = 1.5708;
                                                                            fk();
                                                                            alt_a3 = -1.5708;
                                                                            fk();
                                                                            //alt_a2_claw = -1.5708;
                                                                            //fk();
                                                                            //alt_a2 = 1.5708;
                                                                            //fk();
                                                                          
                                                                            //Update elbow joint
                                                                            startToGoal = alt_goal.minus(alt_start_l2_claw);
                                                                            startToEndEffector = alt_endPoint_claw.minus(alt_start_l2_claw);
                                                                            dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
                                                                            dotProd = clamp(dotProd,-1,1);
                                                                            angleDiff = acos(dotProd);
                                                                            if (cross(startToGoal,startToEndEffector) < 0){
                                                                              alt_a2_claw += angleDiff;
                                                                              alt_a2_claw = min(alt_a2_claw, -1.2708);
                                                                            }else{
                                                                              alt_a2_claw -= angleDiff;
                                                                              alt_a2_claw = max(alt_a2_claw, -1.5708);
                                                                            }
                                                                            fk(); //Update link positions with fk (e.g. end effector changed)
                                                                            
                                                                            //Update elbow joint
                                                                            startToGoal = alt_goal.minus(alt_start_l2);
                                                                            startToEndEffector = alt_endPoint.minus(alt_start_l2);
                                                                            dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
                                                                            dotProd = clamp(dotProd,-1,1);
                                                                            angleDiff = acos(dotProd);
                                                                            if (cross(startToGoal,startToEndEffector) < 0){
                                                                              alt_a2 += angleDiff;
                                                                              alt_a2 = min(alt_a2, 1.5708);
                                                                            }else{
                                                                              alt_a2 -= angleDiff;
                                                                              alt_a2 = max(alt_a2, 1.2708);
                                                                            }
                                                                            fk(); //Update link positions with fk (e.g. end effector changed)
                                                                          
                                                                            //Update elbow joint
                                                                            startToGoal = alt_goal.minus(alt_start_l1);
                                                                            startToEndEffector = alt_endPoint.minus(alt_start_l1);
                                                                            dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
                                                                            dotProd = clamp(dotProd,-1,1);
                                                                            angleDiff = acos(dotProd);
                                                                            if (cross(startToGoal,startToEndEffector) < 0){
                                                                              alt_a1 += angleDiff;
                                                                              alt_a1 = min(alt_a1, 0);
                                                                            }else{
                                                                              alt_a1 -= angleDiff;
                                                                              alt_a1 = max(alt_a1, -1.5708);
                                                                            }
                                                                            fk(); //Update link positions with fk (e.g. end effector changed)
                                                                            
                                                                            //Update shoulder joint
                                                                            startToGoal = alt_goal.minus(alt_root);
                                                                            if (startToGoal.length() < .0001) return;
                                                                            startToEndEffector = alt_endPoint.minus(alt_root);
                                                                            dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
                                                                            dotProd = clamp(dotProd,-1,1);
                                                                            angleDiff = acos(dotProd);
                                                                            if (cross(startToGoal,startToEndEffector) < 0){
                                                                              alt_a0 += angleDiff;
                                                                              alt_a0 = min(alt_a0, 0.4);
                                                                            }else{
                                                                              alt_a0 -= angleDiff;
                                                                              alt_a0 = max(alt_a0, -1.8872);
                                                                            }
                                                                            fk(); //Update link positions with fk (e.g. end effector changed)
}

void fk(){
  start_l1 = new Vec2(cos(a0)*l0,sin(a0)*l0).plus(root);
  start_l2 = new Vec2(cos(a0+a1)*l1,sin(a0+a1)*l1).plus(start_l1);
  start_l3 = new Vec2(cos(a0+a1+a2)*l2,sin(a0+a1+a2)*l2).plus(start_l2);
  endPoint = new Vec2(cos(a0+a1+a2+a3)*l3,sin(a0+a1+a2+a3)*l3).plus(start_l3);
  
  start_l2_claw = new Vec2(cos(a0+a1)*l1,sin(a0+a1)*l1).plus(start_l1);
  start_l3_claw = new Vec2(cos(a0+a1+a2_claw)*l2_claw,sin(a0+a1+a2_claw)*l2_claw).plus(start_l2_claw);
  endPoint_claw = new Vec2(cos(a0+a1+a2_claw+a3_claw)*l3_claw,sin(a0+a1+a2_claw+a3_claw)*l3_claw).plus(start_l3_claw);
  
  
  alt_start_l1 = new Vec2(cos(alt_a0)*alt_l0,sin(alt_a0)*alt_l0).plus(alt_root);
  alt_start_l2 = new Vec2(cos(alt_a0+alt_a1)*alt_l1,sin(alt_a0+alt_a1)*alt_l1).plus(alt_start_l1);
  alt_start_l3 = new Vec2(cos(alt_a0+alt_a1+alt_a2)*alt_l2,sin(alt_a0+alt_a1+alt_a2)*alt_l2).plus(alt_start_l2);
  alt_endPoint = new Vec2(cos(alt_a0+alt_a1+alt_a2+alt_a3)*alt_l3,sin(alt_a0+alt_a1+alt_a2+alt_a3)*alt_l3).plus(alt_start_l3);
  
  alt_start_l2_claw = new Vec2(cos(alt_a0+alt_a1)*alt_l1,sin(alt_a0+alt_a1)*alt_l1).plus(alt_start_l1);
  alt_start_l3_claw = new Vec2(cos(alt_a0+alt_a1+alt_a2_claw)*alt_l2_claw,sin(alt_a0+alt_a1+alt_a2_claw)*alt_l2_claw).plus(alt_start_l2_claw);
  alt_endPoint_claw = new Vec2(cos(alt_a0+alt_a1+alt_a2_claw+alt_a3_claw)*alt_l3_claw,sin(alt_a0+alt_a1+alt_a2_claw+alt_a3_claw)*alt_l3_claw).plus(alt_start_l3_claw);
}

void leg_fk(){
  leg_start_l1 = new Vec2(cos(leg_a0)*leg_l0,sin(leg_a0)*leg_l0).plus(leg_root);
  leg_endPoint = new Vec2(cos(leg_a0+leg_a1)*leg_l1,sin(leg_a0+leg_a1)*leg_l1).plus(leg_start_l1);
  
  alt_leg_start_l1 = new Vec2(cos(alt_leg_a0)*alt_leg_l0,sin(alt_leg_a0)*alt_leg_l0).plus(alt_leg_root);
  alt_leg_endPoint = new Vec2(cos(alt_leg_a0+alt_leg_a1)*alt_leg_l1,sin(alt_leg_a0+alt_leg_a1)*alt_leg_l1).plus(alt_leg_start_l1);
  
  lleg_start_l1 = new Vec2(cos(lleg_a0)*lleg_l0,sin(lleg_a0)*lleg_l0).plus(leg_root);
  lleg_endPoint = new Vec2(cos(lleg_a0+lleg_a1)*lleg_l1,sin(lleg_a0+lleg_a1)*lleg_l1).plus(lleg_start_l1);
  
  lalt_leg_start_l1 = new Vec2(cos(lalt_leg_a0)*lalt_leg_l0,sin(lalt_leg_a0)*lalt_leg_l0).plus(alt_leg_root);
  lalt_leg_endPoint = new Vec2(cos(lalt_leg_a0+lalt_leg_a1)*lalt_leg_l1,sin(lalt_leg_a0+lalt_leg_a1)*lalt_leg_l1).plus(lalt_leg_start_l1);
  
  aleg_start_l1 = new Vec2(cos(aleg_a0)*aleg_l0,sin(aleg_a0)*aleg_l0).plus(leg_root);
  aleg_endPoint = new Vec2(cos(aleg_a0+aleg_a1)*aleg_l1,sin(aleg_a0+aleg_a1)*aleg_l1).plus(aleg_start_l1);
  
  aalt_leg_start_l1 = new Vec2(cos(aalt_leg_a0)*aalt_leg_l0,sin(aalt_leg_a0)*aalt_leg_l0).plus(alt_leg_root);
  aalt_leg_endPoint = new Vec2(cos(aalt_leg_a0+aalt_leg_a1)*aalt_leg_l1,sin(aalt_leg_a0+aalt_leg_a1)*aalt_leg_l1).plus(aalt_leg_start_l1);
}  

boolean hit = false;
void fall(float dt){
  hit = false;
  acc = gravity;
   vel.add(acc.times(dt));
  
  
  
  
   Vec2 v = start_l3_claw.minus(start_l3);
   float v_len = v.length(); //Save the distance of the line
   v.normalize();
  
   hitInfo info = lineCircleIntesect(pos, rad/2, start_l3, v, v_len);
   if(info.hit){
     hit = true;
     vel.y *= -0.5;   
     vel.x *= -0.5;
   }
   
   
   v = endPoint.minus(start_l3);
   v_len = v.length(); //Save the distance of the line
   v.normalize();
  
   info = lineCircleIntesect(pos, rad/2, start_l3, v, v_len);
   if(info.hit){
     hit = true;
     vel.y *= -0.5;   
     vel.x *= -0.5;    
   }
   
   
   v = endPoint_claw.minus(start_l3_claw);
   v_len = v.length(); //Save the distance of the line
   v.normalize();
  
   info = lineCircleIntesect(pos, rad/2, start_l3_claw, v, v_len);
   if(info.hit){
     hit = true;
     vel.y *= -0.5;   
     vel.x *= -0.5; 
   }
   
   
   
   
   
   v = alt_start_l3_claw.minus(alt_start_l3);
   v_len = v.length(); //Save the distance of the line
   v.normalize();
  
   info = lineCircleIntesect(pos, rad/2, alt_start_l3, v, v_len);
   if(info.hit){
     hit = true;
     vel.y *= -0.5;   
     vel.x *= -0.5;
   }
   
   
   v = alt_endPoint.minus(alt_start_l3);
   v_len = v.length(); //Save the distance of the line
   v.normalize();
  
   info = lineCircleIntesect(pos, rad/2, alt_start_l3, v, v_len);
   if(info.hit){
     hit = true;
     vel.y *= -0.5;   
     vel.x *= -0.5;
   }
      
   
   v = alt_endPoint_claw.minus(alt_start_l3_claw);
   v_len = v.length(); //Save the distance of the line
   v.normalize();
  
   info = lineCircleIntesect(pos, rad/2, alt_start_l3_claw, v, v_len);
   if(info.hit){
     hit = true;
     vel.y *= -0.5;   
     vel.x *= -0.5; 
   }
  

   pos.add(vel.times(dt));
   
   if (pos.y+rad/2 > floor){
     vel.y *= -.9;
     pos.y = floor - rad/2;
   }else if(pos.y - rad/2 < 0){
     pos.y = rad/2;
     vel.y *= -.9;
   }
   if (pos.x + rad/2 > 960) {
      pos.x = width - rad/2;
      vel.x *= -0.9;
    }
    else if (pos.x - rad/2 < 0) {
      pos.x = rad/2;
      vel.x *= -0.9;
    }
   
}

void draw(){
  move();
  if((millis() / 1000.0) - startTime >= 0.025){
    if(pause) fall(1/frameRate);
    fk();
    solve();
    startTime = millis() / 1000.0;
  }
  
  
  background(250,250,250);
  if(!hit){
    fill(150,150,250); 
    pushMatrix();
    circle(pos.x,pos.y,rad);
    popMatrix();
  }else{
    pos = new Vec2(random(840)+60,100);
    vel = new Vec2(random(500)+50,random(200)+50); 
  }
  fill(255,88,79);  
  
  pushMatrix();
  translate(body.x,body.y);
  rect(0, -lbody/2, wbody, lbody);
  popMatrix();
  
  
  
  
  
  pushMatrix();
  translate(root.x,root.y);
  rotate(a0);
  rect(0, -armW/2, l0, armW);
  popMatrix();
  
  pushMatrix();
  translate(start_l1.x,start_l1.y);
  rotate(a0+a1);
  rect(0, -armW/2, l1, armW);
  popMatrix();
  
  pushMatrix();
  translate(start_l2.x,start_l2.y);
  rotate(a0+a1+a2);
  rect(0, -clawW/2, l2, clawW);
  popMatrix();
  
  
  
  
  
  
  
  
  pushMatrix();
  translate(start_l2_claw.x,start_l2_claw.y);
  rotate(a0+a1+a2_claw);
  rect(0, -clawW/2, l2_claw, clawW);
  popMatrix();
  
  pushMatrix();
  translate(start_l3.x,start_l3.y);
  rotate(a0+a1+a2+a3);
  rect(0, -clawW/2, l3, clawW);
  popMatrix();
  
  pushMatrix();
  translate(start_l3_claw.x,start_l3_claw.y);
  rotate(a0+a1+a2_claw+a3_claw);
  rect(0, -clawW/2, l3_claw, clawW);
  popMatrix();  
  
  
  pushMatrix();
  translate(alt_root.x,alt_root.y);
  rotate(alt_a0);
  rect(0, -armW/2, alt_l0, armW);
  popMatrix();
  
  pushMatrix();
  translate(alt_start_l1.x,alt_start_l1.y);
  rotate(alt_a0+alt_a1);
  rect(0, -armW/2, alt_l1, armW);
  popMatrix();
  
  pushMatrix();
  translate(alt_start_l2.x,alt_start_l2.y);
  rotate(alt_a0+alt_a1+alt_a2);
  rect(0, -clawW/2, alt_l2, clawW);
  popMatrix();
  
  pushMatrix();
  translate(alt_start_l2_claw.x,alt_start_l2_claw.y);
  rotate(alt_a0+alt_a1+alt_a2_claw);
  rect(0, -clawW/2, alt_l2_claw, clawW);
  popMatrix();
  
  pushMatrix();
  translate(alt_start_l3.x,alt_start_l3.y);
  rotate(alt_a0+alt_a1+alt_a2+alt_a3);
  rect(0, -clawW/2, alt_l3, clawW);
  popMatrix();
  
  pushMatrix();
  translate(alt_start_l3_claw.x,alt_start_l3_claw.y);
  rotate(alt_a0+alt_a1+alt_a2_claw+alt_a3_claw);
  rect(0, -clawW/2, alt_l3_claw, clawW);
  popMatrix();  
  

  pushMatrix();
  translate(root.x,root.y);
  circle(0, -clawW/2, rad);
  popMatrix();
  
  pushMatrix();
  translate(cos(a0)*l0+root.x,sin(a0)*l0+root.y);
  circle(0, -clawW/2, rad);
  popMatrix();
  
  pushMatrix();
  translate(alt_root.x,alt_root.y);
  circle(0, -clawW/2, rad);
  popMatrix();
  
  pushMatrix();
  translate(cos(alt_a0)*alt_l0+alt_root.x,sin(alt_a0)*alt_l0+alt_root.y);
  circle(0, -clawW/2, rad);
  popMatrix();
  
  
  fill(0,0,0);
  
  
  pushMatrix();
  translate(start_l3.x,start_l3.y);
  circle(0, -clawW/2, 5);
  popMatrix();
  
  pushMatrix();
  translate(alt_start_l3.x,alt_start_l3.y);
  circle(0, -clawW/2, 5);
  popMatrix();
  
  pushMatrix();
  translate(start_l3_claw.x,start_l3_claw.y);
  circle(0, -clawW/2, 5);
  popMatrix();
  
  pushMatrix();
  translate(alt_start_l3_claw.x,alt_start_l3_claw.y);
  circle(0, -clawW/2, 5);
  popMatrix();
  
  pushMatrix();
  translate(endPoint.x,endPoint.y);
  circle(0, -clawW/2, 5);
  popMatrix();
  
  pushMatrix();
  translate(endPoint_claw.x,endPoint_claw.y);
  circle(0, -clawW/2, 5);
  popMatrix();

  pushMatrix();
  translate(alt_endPoint.x,alt_endPoint.y);
  circle(0, -clawW/2, 5);
  popMatrix();
  
  pushMatrix();
  translate(alt_endPoint_claw.x,alt_endPoint_claw.y);
  circle(0, -clawW/2, 5);
  popMatrix();
  
  
  
  
  fill(254,132,132);
  pushMatrix();
  translate(leg_root.x,leg_root.y);
  rotate(lleg_a0);
  rect(0, -legW/2, lleg_l0, legW);
  popMatrix();
  
  pushMatrix();
  translate(lleg_start_l1.x,lleg_start_l1.y);
  rotate(lleg_a0+lleg_a1);
  rect(0, -legW/2, lleg_l1, legW);
  popMatrix();
  
  
  pushMatrix();
  translate(alt_leg_root.x,alt_leg_root.y);
  rotate(lalt_leg_a0);
  rect(0, -legW/2, lalt_leg_l0, legW);
  popMatrix();
  
  pushMatrix();
  translate(lalt_leg_start_l1.x,lalt_leg_start_l1.y);
  rotate(lalt_leg_a0+lalt_leg_a1);
  rect(0, -legW/2, lalt_leg_l1, legW);
  popMatrix();
  
  pushMatrix();
  translate(leg_root.x,leg_root.y);
  rotate(leg_a0);
  rect(0, -legW/2, leg_l0, legW);
  popMatrix();
  
  pushMatrix();
  translate(leg_start_l1.x,leg_start_l1.y);
  rotate(leg_a0+leg_a1);
  rect(0, -legW/2, leg_l1, legW);
  popMatrix();
  
  
  pushMatrix();
  translate(alt_leg_root.x,alt_leg_root.y);
  rotate(alt_leg_a0);
  rect(0, -legW/2, alt_leg_l0, legW);
  popMatrix();
  
  pushMatrix();
  translate(alt_leg_start_l1.x,alt_leg_start_l1.y);
  rotate(alt_leg_a0+alt_leg_a1);
  rect(0, -legW/2, alt_leg_l1, legW);
  popMatrix();
  
  
  
  
  
  
  pushMatrix();
  translate(leg_root.x,leg_root.y);
  rotate(aleg_a0);
  rect(0, -legW/2, aleg_l0, legW);
  popMatrix();
  
  pushMatrix();
  translate(aleg_start_l1.x,aleg_start_l1.y);
  rotate(aleg_a0+aleg_a1);
  rect(0, -legW/2, aleg_l1, legW);
  popMatrix();
  
  
  pushMatrix();
  translate(alt_leg_root.x,alt_leg_root.y);
  rotate(aalt_leg_a0);
  rect(0, -legW/2, aalt_leg_l0, legW);
  popMatrix();
  
  pushMatrix();
  translate(aalt_leg_start_l1.x,aalt_leg_start_l1.y);
  rotate(aalt_leg_a0+aalt_leg_a1);
  rect(0, -legW/2, aalt_leg_l1, legW);
  popMatrix();
  
  
  

}


void solve_walk(){
  if((millis() / 1000.0) - walkTime <= 0.15) return;
  if(dex == 0){
    leg_a0 = 3.24159;
    leg_a1 = 1.549542+3.14159;
    
    alt_leg_a0 =  0;
    alt_leg_a1 = 1.549542;
    
    lleg_a0= 3.04159;
    lleg_a1= 1.8045176+3.14159;
    
    lalt_leg_a0= 0.1;
    lalt_leg_a1= 1.8045176;
    
    aleg_a0= 2.74159;
    aleg_a1= 1.2552241+3.14159;
    
    aalt_leg_a0= 0.4;
    aalt_leg_a1= 1.2552241;
  }
  if(dex == 1){
    leg_a0= 3.14159;
    leg_a1= 1.2552241+3.14159;
    
    alt_leg_a0= -0.1;
    alt_leg_a1= 1.2552241;
    
    lleg_a0 = 3.24159;
    lleg_a1 = 1.549542+3.14159;
    
    lalt_leg_a0 =  0;
    lalt_leg_a1 = 1.549542;
    
    aleg_a0= 2.64159;
    aleg_a1= 1.8045176+3.14159;
    
    aalt_leg_a0= 0.5;
    aalt_leg_a1= 1.8045176;
  }
  if(dex == 2){
    leg_a0= 3.04159;
    leg_a1= 1.8045176+3.14159;
    
    alt_leg_a0= 0.1;
    alt_leg_a1= 1.8045176;
    
    lleg_a0= 3.14159;
    lleg_a1= 1.2552241+3.14159;
    
    lalt_leg_a0= -0.1;
    lalt_leg_a1= 1.2552241;
    
    aleg_a0 = 2.84159;
    aleg_a1 = 1.549542+3.14159;
    
    aalt_leg_a0 =  0.4;
    aalt_leg_a1 = 1.549542;
  }
  leg_fk();
  dex = (dex+1)%3;
  walkTime = millis() / 1000.0;
    
}

void move(){
  if( leftPressed ){
    alt_goal.x -= speed;
  }
  if( rightPressed ){
    alt_goal.x += speed;
  }
  if( upPressed ){
    alt_goal.y -= speed;
  }
  if( downPressed ){
    alt_goal.y += speed;
  }    
    
  if( aPressed ){
    goal.x -= speed;
  }
  if( dPressed ){
    goal.x += speed;
  }
  if( wPressed ){
    goal.y -= speed;
  }
  if( sPressed ){
    goal.y += speed;
  }
  
  
  
  if( vPressed ){
    body.x -= speed;
    root.x -= speed;
    alt_root.x -= speed;
    leg_root.x -= speed;
    alt_leg_root.x -= speed;
    leg_fk();
    solve_walk();
  }
  if( bPressed ){
    body.x+= speed;
    root.x += speed;
    alt_root.x += speed;
    leg_root.x += speed;
    alt_leg_root.x += speed;
    leg_fk();
    solve_walk();
  }
  
}

boolean leftPressed = false, rightPressed = false, upPressed = false, downPressed = false;
boolean aPressed = false, dPressed = false, wPressed = false, sPressed = false;
boolean fToggle = false, gToggle = false;
boolean vPressed = false, bPressed = false;
boolean pause = false;
void keyPressed(){
  if ( key == 'r' ){
    pos = new Vec2(random(840)+60,100);
    vel = new Vec2(random(100)+50,random(100)+50); 
  }
  if ( key == ' ' ){
    pause = !pause; 
  }
  if ( key == 'f' ){
    fToggle = !fToggle;
  }
  if ( key == 'g' ){
    gToggle = !gToggle;
  }
  
  if ( key == 'v' ){
    vPressed = true;
  }
  if ( key == 'b' ){
    bPressed = true;
  }
  
  if ( keyCode == LEFT ){
    leftPressed = true;
  }
  if ( keyCode == RIGHT ){
    rightPressed = true;
  }
  if ( keyCode == UP ){
    upPressed = true;
  }
  if ( keyCode == DOWN ){
    downPressed = true;
  }
  
  if ( key == 'a' ){
    aPressed = true;
  }
  if ( key == 'd' ){
    dPressed = true;
  }
  if ( key == 'w' ){
    wPressed = true;
  }
  if ( key == 's' ){
    sPressed = true;
  }
}

void keyReleased(){
  if ( key == 'v' ){
    vPressed = false;
  }
  if ( key == 'b' ){
    bPressed = false;
  }
  
  if ( keyCode == LEFT ){
    leftPressed = false;
  }
  if ( keyCode == RIGHT ){
    rightPressed = false;
  }
  if ( keyCode == UP ){
    upPressed = false;
  }
  if ( keyCode == DOWN ){
    downPressed = false;
  }
  
  if ( key == 'a' ){
    aPressed = false;
  }
  if ( key == 'd' ){
    dPressed = false;
  }
  if ( key == 'w' ){
    wPressed = false;
  }
  if ( key == 's' ){
    sPressed = false;
  }
}


//-----------------
// Vector Library
//-----------------

//Vector Library
//CSCI 5611 Vector 2 Library [Example]
// Stephen J. Guy <sjguy@umn.edu>

public class Vec2 {
  public float x, y;
  
  public Vec2(float x, float y){
    this.x = x;
    this.y = y;
  }
  
  public String toString(){
    return "(" + x+ "," + y +")";
  }
  
  public float length(){
    return sqrt(x*x+y*y);
  }
  
  public float lengthSqr(){
    return x*x+y*y;
  }
  
  public Vec2 plus(Vec2 rhs){
    return new Vec2(x+rhs.x, y+rhs.y);
  }
  
  public void add(Vec2 rhs){
    x += rhs.x;
    y += rhs.y;
  }
  
  public Vec2 minus(Vec2 rhs){
    return new Vec2(x-rhs.x, y-rhs.y);
  }
  
  public void subtract(Vec2 rhs){
    x -= rhs.x;
    y -= rhs.y;
  }
  
  public Vec2 times(float rhs){
    return new Vec2(x*rhs, y*rhs);
  }
  
  public void mul(float rhs){
    x *= rhs;
    y *= rhs;
  }
  
  public void clampToLength(float maxL){
    float magnitude = sqrt(x*x + y*y);
    if (magnitude > maxL){
      x *= maxL/magnitude;
      y *= maxL/magnitude;
    }
  }
  
  public void setToLength(float newL){
    float magnitude = sqrt(x*x + y*y);
    x *= newL/magnitude;
    y *= newL/magnitude;
  }
  
  public void normalize(){
    float magnitude = sqrt(x*x + y*y);
    x /= magnitude;
    y /= magnitude;
  }
  
  public Vec2 normalized(){
    float magnitude = sqrt(x*x + y*y);
    return new Vec2(x/magnitude, y/magnitude);
  }
  
  public float distanceTo(Vec2 rhs){
    float dx = rhs.x - x;
    float dy = rhs.y - y;
    return sqrt(dx*dx + dy*dy);
  }
}

Vec2 interpolate(Vec2 a, Vec2 b, float t){
  return a.plus((b.minus(a)).times(t));
}

float interpolate(float a, float b, float t){
  return a + ((b-a)*t);
}

float dot(Vec2 a, Vec2 b){
  return a.x*b.x + a.y*b.y;
}

float cross(Vec2 a, Vec2 b){
  return a.x*b.y - a.y*b.x;
}


Vec2 projAB(Vec2 a, Vec2 b){
  return b.times(a.x*b.x + a.y*b.y);
}

float clamp(float f, float min, float max){
  if (f < min) return min;
  if (f > max) return max;
  return f;
}
