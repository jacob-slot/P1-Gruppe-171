void Challenge0(){
if (encoders.getCountsRight() > 50)
  {
    scrollValue = constrain(scrollValue - 1, 0, listLength-2);
    encoders.getCountsAndResetRight();
    displayChange("PickProgram", programs[scrollValue], programs[scrollValue+1], programs[scrollValue+2]);
  }
  else if(encoders.getCountsRight() < -50)
  {
    scrollValue = constrain(scrollValue + 1, 0, listLength-2);
    encoders.getCountsAndResetRight();
    displayChange("PickProgram", programs[scrollValue], programs[scrollValue+1], programs[scrollValue+2]);
  }
  
  if(buttonA.getSingleDebouncedRelease())
  {
    encoders.getCountsAndResetRight();
    runProgram();
  }

}

void displayChange(char* Header, char* word1, char* word2, char* word3)
{
  display.clear();
  display.gotoXY(0,0);
  display.print(Header);
  display.gotoXY(0,1);
  display.print("\2 ");
  display.print(word1);
  display.gotoXY(0,2);
  display.print("- ");
  display.print(word2);
  display.gotoXY(0,3);
  display.print("\1 ");
  display.print(word3);
}

void runProgram()
{
  switch(scrollValue)
  {
    case 0:
      Program1();
    break;
    case 1:
      Program2();
    break;
    case 2:
      Program3();
    break;
    case 3:
      Program4();
    break;
    case 4:
      Program5();
    break;
    case 5:
      Program6();
    break;
    case 6:
      Program7();
    break;

}

  }
