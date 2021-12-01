/*
 * PageDisplay.c
 *
 *  Created on: Nov 28, 2021
 *      Author: Dhruv
 */

#include "PageDisplay.h"
#define INCLUDE_LOG_DEBUG 1
#include "src/log.h"

 char *Book1 =
    "Simple Moscow Mule\n"
    "Ingredients\n"
    "Ice\n"
    "1/2 Fresh Lime\n"
    "2 fl oz vodka\n"
    "4 fl oz ginger beer\n"
    "2 lime slices\n"
    "Fill a tall glass with ice\n"
    "Squeeze lime. Pour vodka over ice. Top with ginger beer. Garnish with lime slices\n";

char *currentPos;

void PrintDisplay()
{
  char *newline = strchr(currentPos, '\n');
  char *oldline = currentPos;
  int row = 0, sameLineFlag = 0;
  char singleLine[20];
  static int doOnce = 1;

  if(doOnce)
    {
      currentPos = Book1;
      doOnce = 0;
    }

    while(newline != NULL)
      {
        memset(singleLine, '\0', sizeof(singleLine)/sizeof(char));

        if(newline - oldline > DISPLAY_ROW_LEN - 2)
          {
            newline = oldline + DISPLAY_ROW_LEN - 2;
            sameLineFlag = 1;
          }

        strncpy(singleLine, currentPos + (oldline - currentPos), newline - oldline);
        LOG_INFO("singleLine = %s\r\n", singleLine);
        displayPrintf(row++, singleLine);

        if(row == DISPLAY_NUMBER_OF_ROWS - 1)
          break;

        if(sameLineFlag)
          {
            oldline = newline;
            sameLineFlag = 0;
          }

        else oldline = newline + 1;

        newline = strchr(currentPos + (oldline - currentPos), '\n');
      }

}

void scrollUp()
{
  char *oldline = currentPos;
  char *newline = strchr(currentPos, '\n');
  int lineCount = 0, sameLineFlag = 0;

  while(lineCount != 2 && newline != NULL)
    {
      if(newline - oldline > DISPLAY_ROW_LEN - 2)
        {
          newline = oldline + DISPLAY_ROW_LEN - 2;
          sameLineFlag = 1;
        }

      if(sameLineFlag)
        {
          oldline = newline;
          sameLineFlag = 0;
        }

      else oldline = newline + 1;

      newline = strchr(Book1 + (oldline - Book1), '\n');
      lineCount++;
    }

    currentPos = newline + 1;

    for(int i = 0; i < DISPLAY_NUMBER_OF_ROWS - 1; i++)
      displayPrintf(i, " ");
}
