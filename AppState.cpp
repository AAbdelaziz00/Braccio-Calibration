/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include "AppState.h"
#include "IK_FK.h" 

/**************************************************************************************
 * DEFINE
 **************************************************************************************/

#define COLOR_TEAL       0x00878F
#define COLOR_LIGHT_TEAL 0x62AEB2
#define COLOR_ORANGE     0xE47128

#define BUTTON_ENTER     6

/**************************************************************************************
 * CONSTANTS
 **************************************************************************************/

static float const MIN_POS[6] =  {157.5,   0.0,  80.0,  75.0,  60.0,   0.0};
static float const MAX_POS[6] =  {230.0, 315.0, 230.0, 225.0, 250.0, 315.0};
static float const HOME_POS[6] = {157.5, 157.5, 157.5, 157.5, 157.5,   0.0};

/* Joint position value for the calibration of the base (M6) */ 
static float const M6_CALIBRATION_POSITION = 90.0;

/**************************************************************************************
 * GLOBAL VARIABLES
 **************************************************************************************/

lv_obj_t * txtLbl;
lv_obj_t * btnm; 
const char * btnm_map[] = {
  " ", "\n", " ", "\n", // Hidden buttons. Added to provide blank space for text label
  "ZERO_POSITION", "\n", "CALIBRATE", "\n", "TEST CALIBRATION", "\n", 
  "TEST FORWARD_KIN", "\n", "TEST INVERSE_KIN", "\n", "PICK_&_PLACE", "\n",
  "BACK", "NEXT", "\n", 
  "\0" };
const int BTN_ID[8] = {2, 3, 4, 5, 6, 7, 8, 9};
const int NEXT_BTN_ID = BTN_ID[7];
 
static int  step_count;

extern CalibrateAndTestApp app; 

/**************************************************************************************
 * FUNCTION DEFINITION
 **************************************************************************************/

static void event_handler_menu(lv_event_t * e)
{
  lv_event_code_t code = lv_event_get_code(e);

  if (code == LV_EVENT_CLICKED || (code == LV_EVENT_KEY && Braccio.getKey() == BUTTON_ENTER))
  {
    lv_obj_t * obj = lv_event_get_target(e);
    uint32_t const id = lv_btnmatrix_get_selected_btn(obj);

    switch (id)
    {  
    case 2: app.update(EventSource::Button_ZeroPosition);       break;
    case 3: app.update(EventSource::Button_Calibrate);          break;
    case 4: app.update(EventSource::Button_Calibration_Test);   break;
    case 5: app.update(EventSource::Button_Forward_Kinematics); break; 
    case 8: app.update(EventSource::Button_Back);               break; 
    case 9: app.update(EventSource::Button_Next);               break; 
    }
  }
}

void custom_main_menu()
{
  Braccio.lvgl_lock();  

  static lv_style_t style_focus;
  lv_style_init(&style_focus);
  lv_style_set_outline_color(&style_focus, lv_color_hex(COLOR_ORANGE));
  lv_style_set_outline_width(&style_focus, 4);

  static lv_style_t style_btn;
  lv_style_init(&style_btn);
  lv_style_set_bg_color(&style_btn, lv_color_hex(COLOR_LIGHT_TEAL));
  lv_style_set_text_color(&style_btn, lv_color_white());

  btnm = lv_btnmatrix_create(lv_scr_act());
  lv_btnmatrix_set_map(btnm, btnm_map);
  lv_obj_set_size(btnm, 240, 240);
  lv_obj_align(btnm, LV_ALIGN_CENTER, 0, 0); 

  lv_obj_add_style(btnm, &style_btn, LV_PART_ITEMS);
  lv_obj_add_style(btnm, &style_focus, LV_PART_ITEMS | LV_STATE_FOCUS_KEY);

  lv_btnmatrix_set_btn_ctrl(btnm, 0, LV_BTNMATRIX_CTRL_HIDDEN);
  lv_btnmatrix_set_btn_ctrl(btnm, 1, LV_BTNMATRIX_CTRL_HIDDEN); 
  lv_btnmatrix_set_btn_ctrl(btnm, BTN_ID[0], LV_BTNMATRIX_CTRL_DISABLED);
  lv_btnmatrix_set_btn_ctrl(btnm, BTN_ID[1], LV_BTNMATRIX_CTRL_DISABLED);
  lv_btnmatrix_set_btn_ctrl(btnm, BTN_ID[2], LV_BTNMATRIX_CTRL_DISABLED);
  lv_btnmatrix_set_btn_ctrl(btnm, BTN_ID[3], LV_BTNMATRIX_CTRL_DISABLED);
  lv_btnmatrix_set_btn_ctrl(btnm, BTN_ID[4], LV_BTNMATRIX_CTRL_HIDDEN);
  lv_btnmatrix_set_btn_ctrl(btnm, BTN_ID[5], LV_BTNMATRIX_CTRL_HIDDEN);
  lv_btnmatrix_set_btn_ctrl(btnm, BTN_ID[6], LV_BTNMATRIX_CTRL_HIDDEN);
  lv_btnmatrix_set_btn_ctrl(btnm, BTN_ID[7], LV_BTNMATRIX_CTRL_HIDDEN);

  lv_btnmatrix_set_one_checked(btnm, true); 

  lv_obj_add_event_cb(btnm, event_handler_menu, LV_EVENT_ALL, NULL);

  txtLbl = lv_label_create(btnm); 
  lv_label_set_text(txtLbl," "); 
  lv_obj_align(txtLbl, LV_ALIGN_TOP_MID, 0, 0); 

  Braccio.lvgl_unlock();

  Braccio.connectJoystickTo(btnm);
}

void moveArmToPos(const float* requested_pos){ 
  bool move_is_safe = true; 
  if(requested_pos[5]>180||requested_pos[5]<0||requested_pos[4]>180||requested_pos[3]>180||requested_pos[2]>180){
    Serial.println("Unsafe joint angles. Base: "+String(requested_pos[5])+", Shoulder: "+String(requested_pos[4])+
                                      ", Elbow: "+String(requested_pos[3])+", Wrist Roll: "+String(requested_pos[2])+"\n");
    move_is_safe = false; 
  }
  float tb=requested_pos[5],ts=requested_pos[4]+22.5,te=requested_pos[3]+22.5,tw=requested_pos[2]+22.5;
  float b5=(180.0-ts)*0.017453,b4=(360.0-ts-te)*0.017453,b3=(540.0-ts-te-tw)*0.017453;
  float p1=(12.5*(sin(b5)+sin(b4))+19.5*sin(b3))*cos(tb*0.017453),p2=(12.5*(sin(b5)+sin(b4))+19.5*sin(b3))*sin(tb*0.017453);
  float p3=7.2+12.5*(cos(b5)+cos(b4))+19.5*cos(b3);
  if(abs(p1)>21.0||p2<0.0||p3<0.0){
    Serial.println("Unsafe gripper position. (X,Y,Z) = ("+String(p1)+", "+String(p2)+", "+String(p3)+")\n");
    Serial.println("Joint angles -> Base: "+String(requested_pos[5])+", Shoulder: "+String(requested_pos[4])+
                                      ", Elbow: "+String(requested_pos[3])+", Wrist Roll: "+String(requested_pos[2])+"\n");
    move_is_safe = false;
  }
  if(move_is_safe == false){
    lv_label_set_text(txtLbl, "Unsafe joint command!\n\nPress BACK to exit");
    Serial.println("Unsafe joint command!\n\nPress BACK to exit");
    lv_btnmatrix_set_btn_ctrl(btnm, NEXT_BTN_ID, LV_BTNMATRIX_CTRL_HIDDEN);
    return;
  }
  float joint_pos [SmartServoClass::NUM_MOTORS] = {0};
  /* ensure requested angles are within thier stated min and max values */
  for (size_t i = 0; i < SmartServoClass::NUM_MOTORS; i++){
    joint_pos[i] = max(requested_pos[i] + JOINT_OFFSETS[i], MIN_POS[i]);
    joint_pos[i] = min(joint_pos[i], MAX_POS[i]);
  } 

  delay(100);
  Braccio.engage();
  delay(100);

  Braccio.speed(speed_grade_t::SLOW);
  Braccio.moveTo(joint_pos[0], joint_pos[1], joint_pos[2], 
                 joint_pos[3], joint_pos[4], joint_pos[5]);

  auto isInPos = [](float const joint_pos_var[SmartServoClass::NUM_MOTORS]) -> bool
  {
    float current_angles[SmartServoClass::NUM_MOTORS] = {0};
    Braccio.positions(current_angles);

    float total_angle_err = 0.0;
    for (size_t i = 0; i < SmartServoClass::NUM_MOTORS; i++)
      total_angle_err += fabs(current_angles[i] - joint_pos_var[i]);

    static float const TOTAL_EPSILON = 0.50f;
    bool const is_in_pos = (total_angle_err < TOTAL_EPSILON);
    return is_in_pos;
  };
  auto isTimeout = [](unsigned long const start) -> bool
  {
    /* Timeout of one second. */
    auto const now = millis();
    if ((now - start) > 1500)
      return true;
    else
      return false;
  };
  
  auto isJointInPos = [](int const jointID, float const joint_pos_var) -> bool
  {
    float current_angle = Braccio.get(jointID).position(); 

    static float const TOTAL_EPSILON = 0.25f;
    bool const is_in_pos = (fabs(current_angle - joint_pos_var) < TOTAL_EPSILON);
    return is_in_pos;
  };

  auto isJointTimeout = [](unsigned long const start) -> bool
  {
    // Timeout of one second. 
    auto const now = millis();
    if ((now - start) > 1000)
      return true;
    else
      return false;
  }; 
   

  /* Wait until we've returned to the home position
  * with a timeout (i.e. we leave this function)
  * after one second even if we can't fully reach
  * the home position.
  */
  for(auto start = millis(); !isInPos(joint_pos) && !isTimeout(start); delay(100)) { }
  /*
  for(int jntN = 2; jntN < SmartServoClass::NUM_MOTORS; jntN++){
    float jointN_pos = joint_pos[jntN];
    for(auto start = millis(); !isJointInPos(jntN+1, jointN_pos) && !isJointTimeout(start); delay(100)) { }
  } */ 

  float current_angles[SmartServoClass::NUM_MOTORS] = {0};
  Braccio.positions(current_angles);

  // Print the joint angles 
    Serial.println("\n**************************************** Joints Angles **********************************");
    Serial.println("|\tMotor ID\t|\tAngle\t|\tRequested\t|\tPosition error\t|");
    Serial.println("-----------------------------------------------------------------------------------------");
    Serial.print("| 1 - Gripper\t\t|\t" +       String(current_angles[0]) + "\t|\t" + String(joint_pos[0]) + "\t\t|\t" + String(current_angles[0] - joint_pos[0]) + "\t\t|\n" + 
                  "| 2 - Wrist Rotation\t|\t" + String(current_angles[1]) + "\t|\t" + String(joint_pos[1]) + "\t\t|\t" + String(current_angles[1] - joint_pos[1]) + "\t\t|\n" +
                  "| 3 - Wrist Vertical\t|\t" + String(current_angles[2]) + "\t|\t" + String(joint_pos[2]) + "\t\t|\t" + String(current_angles[2] - joint_pos[2]) + "\t\t|\n" + 
                  "| 4 - Elbow\t\t|\t" +    String(current_angles[3]) + "\t|\t" + String(joint_pos[3]) + "\t\t|\t" + String(current_angles[3] - joint_pos[3]) + "\t\t|\n" + 
                  "| 5 - Shoulder\t\t|\t" + String(current_angles[4]) + "\t|\t" + String(joint_pos[4]) + "\t\t|\t" + String(current_angles[4] - joint_pos[4]) + "\t\t|\n" + 
                  "| 6 - Base\t\t|\t" +     String(current_angles[5]) + "\t|\t" + String(joint_pos[5]) + "\t\t|\t" + String(current_angles[5] - joint_pos[5]) + "\t\t|\n");
    Serial.println("-----------------------------------------------------------------------------------------\n");

}

/**************************************************************************************
 * IdleState
 **************************************************************************************/

void IdleState::onEnter()
{ 
  lv_btnmatrix_clear_btn_ctrl(btnm, BTN_ID[0], LV_BTNMATRIX_CTRL_HIDDEN);
  lv_btnmatrix_clear_btn_ctrl(btnm, BTN_ID[1], LV_BTNMATRIX_CTRL_HIDDEN);
  lv_btnmatrix_clear_btn_ctrl(btnm, BTN_ID[2], LV_BTNMATRIX_CTRL_HIDDEN);
  lv_btnmatrix_clear_btn_ctrl(btnm, BTN_ID[3], LV_BTNMATRIX_CTRL_HIDDEN); 
  lv_btnmatrix_set_btn_ctrl(btnm, BTN_ID[6], LV_BTNMATRIX_CTRL_HIDDEN);
  lv_btnmatrix_set_btn_ctrl(btnm, BTN_ID[7], LV_BTNMATRIX_CTRL_HIDDEN);   
}

void IdleState::onExit() { } 

State * IdleState::handle_OnZeroPosition()
{
  return new ZeroState();
} 

State * IdleState::handle_OnCalibrate()
{  
  return new CalibrateState();
} 

State * IdleState::handle_OnCalibrationTest()
{  
  return new CalibrationTestState();
}  

State * IdleState::handle_OnForward_Kinematics()
{  
  return new ForwardKinematicsState();
} 

/**************************************************************************************
 * ZeroState
 **************************************************************************************/

State * ZeroState::handle_OnTimerTick()
{
  return new IdleState();
}

void ZeroState::onEnter()
{
  lv_label_set_text(txtLbl, "Moving to HOME position");

  lv_btnmatrix_set_btn_ctrl(btnm, BTN_ID[0], LV_BTNMATRIX_CTRL_DISABLED);
  lv_btnmatrix_set_btn_ctrl(btnm, BTN_ID[1], LV_BTNMATRIX_CTRL_DISABLED);
  lv_btnmatrix_set_btn_ctrl(btnm, BTN_ID[2], LV_BTNMATRIX_CTRL_DISABLED);
  lv_btnmatrix_set_btn_ctrl(btnm, BTN_ID[3], LV_BTNMATRIX_CTRL_DISABLED);  

  moveArmToPos(HOME_POS);
}
 
void ZeroState::onExit()
{
  lv_label_set_text(txtLbl, " ");

  lv_btnmatrix_clear_btn_ctrl(btnm, BTN_ID[0], LV_BTNMATRIX_CTRL_DISABLED);
  lv_btnmatrix_clear_btn_ctrl(btnm, BTN_ID[1], LV_BTNMATRIX_CTRL_DISABLED);
  lv_btnmatrix_clear_btn_ctrl(btnm, BTN_ID[2], LV_BTNMATRIX_CTRL_DISABLED);
  lv_btnmatrix_clear_btn_ctrl(btnm, BTN_ID[3], LV_BTNMATRIX_CTRL_DISABLED);    
}

/**************************************************************************************
 * CalibrateState
 **************************************************************************************/

void CalibrateState::onEnter()
{   
  lv_btnmatrix_set_btn_ctrl(btnm, BTN_ID[0], LV_BTNMATRIX_CTRL_HIDDEN);
  lv_btnmatrix_set_btn_ctrl(btnm, BTN_ID[1], LV_BTNMATRIX_CTRL_HIDDEN);
  lv_btnmatrix_set_btn_ctrl(btnm, BTN_ID[2], LV_BTNMATRIX_CTRL_HIDDEN);
  lv_btnmatrix_set_btn_ctrl(btnm, BTN_ID[3], LV_BTNMATRIX_CTRL_HIDDEN); 
  lv_btnmatrix_clear_btn_ctrl(btnm, BTN_ID[6], LV_BTNMATRIX_CTRL_HIDDEN);
  lv_btnmatrix_clear_btn_ctrl(btnm, BTN_ID[7], LV_BTNMATRIX_CTRL_HIDDEN); 
  
  delay(100);
  Braccio.disengage();
  delay(100);

  lv_label_set_text(txtLbl, "Step 1 of 2 \n\nMove tip of gripper to \n point B on the center line\n then press NEXT\n\nPress BACK to exit");
}

void CalibrateState::onExit()
{ 
  delay(100);
  Braccio.engage();
  delay(100);
} 

State * CalibrateState::handle_OnBack()
{
  lv_label_set_text(txtLbl, "Settings not saved!");
  return new IdleState(); 
} 

State * CalibrateState::handle_OnNext()
{  
  if(_step_count == 0){
    // store calibration angle of base joint
    _joint_positions[6-1] = Braccio.get(6).position();
    _step_count = _step_count + 1;
    lv_label_set_text(txtLbl, "Step 2 of 2 \n\nSet arm in upright pose\n then press NEXT\n\nPress BACK to exit");

  } else if(_step_count == 1){
    lv_label_set_text(txtLbl, "Saving settings...");
    delay(100);
    Braccio.engage();
    delay(100); 
    // store calibration angles of shoulder, elbow, and wrist joints
    for(int i = 1; i < 6; i++ ){
      _joint_positions[i-1] = Braccio.get(i).position();
    }

    // update all joint offsets except for the gripper (M1 -> i == 0)
    JOINT_OFFSETS[0] = 0.0;
    for(int i = 1; i < 5; i++ ){
      JOINT_OFFSETS[i] = _joint_positions[i] - HOME_POS[i];
    }
    // set joint offset for the base (M6)
    JOINT_OFFSETS[5] = _joint_positions[5] - M6_CALIBRATION_POSITION;

    lv_label_set_text(txtLbl, "Settings saved!");

    // Output results to serial monitor
    float* joint_plus_offset = new float[6];
    for(int i = 0; i < 6; i++ ){
      joint_plus_offset[i] = HOME_POS[i] + JOINT_OFFSETS[i];
    }
      
    // Print the joint angles
    Serial.println("************************* Update joint offsets in line 26 of IK_FK.h ********************\n"); 
    Serial.println("static float JOINT_OFFSETS[6] = {" + String(JOINT_OFFSETS[0]) + ", " + String(JOINT_OFFSETS[1]) + ", " +
                                                         String(JOINT_OFFSETS[2]) + ", " + String(JOINT_OFFSETS[3]) + ", " +
                                                         String(JOINT_OFFSETS[4]) + ", " + String(JOINT_OFFSETS[5]) + "};\n\n");
    Serial.println("**************************************** Joints Angles **********************************");
    Serial.println("|\tMotor ID\t|\tAngle\t|\tOffset\t\t|\tJoint + Offset\t|");
    Serial.println("-----------------------------------------------------------------------------------------");
    Serial.print( "| 1 - Gripper\t\t|\t" +       String(_joint_positions[0]) + "\t|\t" + String(JOINT_OFFSETS[0]) + "\t\t|\t" + String(joint_plus_offset[0]) + "\t\t|\n" + 
                  "| 2 - Wrist Rotation\t|\t" + String(_joint_positions[1]) + "\t|\t" + String(JOINT_OFFSETS[1]) + "\t\t|\t" + String(joint_plus_offset[1]) + "\t\t|\n" +
                  "| 3 - Wrist Vertical\t|\t" + String(_joint_positions[2]) + "\t|\t" + String(JOINT_OFFSETS[2]) + "\t\t|\t" + String(joint_plus_offset[2]) + "\t\t|\n" + 
                  "| 4 - Elbow\t\t|\t" +    String(_joint_positions[3]) + "\t|\t" + String(JOINT_OFFSETS[3]) + "\t\t|\t" + String(joint_plus_offset[3]) + "\t\t|\n" + 
                  "| 5 - Shoulder\t\t|\t" + String(_joint_positions[4]) + "\t|\t" + String(JOINT_OFFSETS[4]) + "\t\t|\t" + String(joint_plus_offset[4]) + "\t\t|\n" + 
                  "| 6 - Base\t\t|\t" +     String(_joint_positions[5]) + "\t|\t" + String(JOINT_OFFSETS[5]) + "\t\t|\t" + String(joint_plus_offset[5]) + "\t\t|\n");
    Serial.println("-----------------------------------------------------------------------------------------");

    return new IdleState();
  } 
  return this;
} 

/**************************************************************************************
 * CalibrationTestState
 **************************************************************************************/

void CalibrationTestState::onEnter()
{   
  lv_btnmatrix_set_btn_ctrl(btnm, BTN_ID[0], LV_BTNMATRIX_CTRL_HIDDEN);
  lv_btnmatrix_set_btn_ctrl(btnm, BTN_ID[1], LV_BTNMATRIX_CTRL_HIDDEN);
  lv_btnmatrix_set_btn_ctrl(btnm, BTN_ID[2], LV_BTNMATRIX_CTRL_HIDDEN);
  lv_btnmatrix_set_btn_ctrl(btnm, BTN_ID[3], LV_BTNMATRIX_CTRL_HIDDEN); 
  lv_btnmatrix_clear_btn_ctrl(btnm, BTN_ID[6], LV_BTNMATRIX_CTRL_HIDDEN);
  lv_btnmatrix_clear_btn_ctrl(btnm, BTN_ID[7], LV_BTNMATRIX_CTRL_HIDDEN); 

  lv_label_set_text_fmt(txtLbl, "Calibration position %d of %d\n press NEXT to proceed\n\nPress BACK to exit", _step_count+1, _total_steps);
  float* fk_angles = new float[6];
  for (size_t i = 0; i < SmartServoClass::NUM_MOTORS; i++){
    fk_angles[i] = CALIBRATION_TEST_POSITIONS[_step_count][i];
  }
  moveArmToPos(fk_angles);
  // update step counter
  _step_count = _step_count + 1;
}

void CalibrationTestState::onExit()
{ 
  lv_label_set_text(txtLbl, " ");
} 

State * CalibrationTestState::handle_OnBack()
{
  return new IdleState();
} 

State * CalibrationTestState::handle_OnNext()
{ 
  if(_step_count == _total_steps){  
    return new IdleState();
  }else{ 
    lv_label_set_text_fmt(txtLbl, "Calibration position %d of %d\n press NEXT to proceed\n\nPress BACK to exit", _step_count+1, _total_steps);
  } 
  float* fk_angles = new float[6];
  for (size_t i = 0; i < SmartServoClass::NUM_MOTORS; i++){
    fk_angles[i] = CALIBRATION_TEST_POSITIONS[_step_count][i];
  }
  moveArmToPos(fk_angles); 
  // update step counter
  _step_count = _step_count + 1;   
  return this;
} 

/**************************************************************************************
 * ForwardKinematicsState
 **************************************************************************************/

void ForwardKinematicsState::onEnter()
{   
  lv_btnmatrix_set_btn_ctrl(btnm, BTN_ID[0], LV_BTNMATRIX_CTRL_HIDDEN);
  lv_btnmatrix_set_btn_ctrl(btnm, BTN_ID[1], LV_BTNMATRIX_CTRL_HIDDEN);
  lv_btnmatrix_set_btn_ctrl(btnm, BTN_ID[2], LV_BTNMATRIX_CTRL_HIDDEN);
  lv_btnmatrix_set_btn_ctrl(btnm, BTN_ID[3], LV_BTNMATRIX_CTRL_HIDDEN); 
  lv_btnmatrix_clear_btn_ctrl(btnm, BTN_ID[6], LV_BTNMATRIX_CTRL_HIDDEN);
  lv_btnmatrix_set_btn_ctrl(btnm, BTN_ID[7], LV_BTNMATRIX_CTRL_HIDDEN); 
  
  delay(100);
  Braccio.disengage();
  delay(100);

  lv_label_set_text(txtLbl, " ");
}

void ForwardKinematicsState::onExit()
{ 
  delay(100);
  Braccio.engage();
  delay(100);
  lv_label_set_text(txtLbl, " ");
} 

State * ForwardKinematicsState::handle_OnBack()
{ 
  return new IdleState();
} 

State * ForwardKinematicsState::handle_OnTimerTick()
{ 
  // get joint positions
  float current_angles[SmartServoClass::NUM_MOTORS] = {0};
  Braccio.positions(current_angles);
  // Account for joint offsets
  for (size_t i = 0; i < SmartServoClass::NUM_MOTORS; i++){
    current_angles[i] = max(current_angles[i] - JOINT_OFFSETS[i], MIN_POS[i]);
    current_angles[i] = min(current_angles[i], MAX_POS[i]);
  }
  // get position of gripper using forward kinematics
  float* fk_pos = forward_kinematics(current_angles);

   Serial.println("Current angles   M1 - Gripper: " + String(current_angles[0]) + 
                  ", M2 - Wrist Rotation: " + String(current_angles[1]) + 
                  ", M3 - Wrist Vertical: " + String(current_angles[2]) + 
                  ", M4 - Elbow: " + String(current_angles[3]) + 
                  ", M5 - Shoulder: " + String(current_angles[4]) + 
                  ", M6 - Base: " + String(current_angles[5]));
   Serial.println("Current position X: " + String(fk_pos[0]) + ", Y: " + String(fk_pos[1]) + 
                                 ", Z: " + String(fk_pos[2])+ "\n");

  // display results
  std::string str = "M3 - Wrist Vertical: \t" + std::to_string((int)current_angles[2]) + 
                  "\nM4 - Elbow: \t" + std::to_string((int)current_angles[3]) + 
                  "\nM5 - Shoulder: \t" + std::to_string((int)current_angles[4]) + 
                  "\nM6 - Base: \t" + std::to_string((int)current_angles[5]) + "\n" +
                  "\nX: \t" + std::to_string((int)fk_pos[0]) +
                  "\nY: \t" + std::to_string((int)fk_pos[1]) +
                  "\nZ: \t" + std::to_string((int)fk_pos[2]);
  char char_arr[str.length() + 1]; 
	strcpy(char_arr, str.c_str());

  lv_label_set_text_fmt(txtLbl, char_arr); 
  return this;
} 

/**************************************************************************************
 * CalibrateAndTestApp
 **************************************************************************************/

void CalibrateAndTestApp::enableButtons()
{
  /* Enable buttons once init is complete. */
  lv_btnmatrix_clear_btn_ctrl(btnm, BTN_ID[0], LV_BTNMATRIX_CTRL_DISABLED);
  lv_btnmatrix_clear_btn_ctrl(btnm, BTN_ID[1], LV_BTNMATRIX_CTRL_DISABLED);
  lv_btnmatrix_clear_btn_ctrl(btnm, BTN_ID[2], LV_BTNMATRIX_CTRL_DISABLED);
  lv_btnmatrix_clear_btn_ctrl(btnm, BTN_ID[3], LV_BTNMATRIX_CTRL_DISABLED); 
}