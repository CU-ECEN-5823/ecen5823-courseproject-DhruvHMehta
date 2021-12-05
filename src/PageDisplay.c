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
    "Squeeze lime. Pour vodka over ice. Top with ginger beer. Garnish with lime slices\n"
    "\n"
    "Easy Apple Cider\n"
    "64 fl oz apple cider\n"
    "3 cinnamon sticks\n"
    "1tsp allspice\n"
    "1tsp cl0oves\n"
    "1/3 cup brown sugar\n"
    "In a slow cooker, combine apple cider and cinnamon sticks."
    " Wrap allspice and cloves in a small piece of cheesecloth,"
    " and add to pot. Stir in brown sugar. Bring to a boil over high heat."
    " Reduce heat, and keep warm\n";

char *currentPos;
uint8_t pageCount;
char *pageBreak[sizeof(char *)*64];

static void revString(char str[])
{
// Find length of string
int len = strlen(str);
// Initialize variable
int i;

// Swap ith to (len - i -1)th character
for(i=0; i < len/2; ++i)
{
//Use str[len] as temp variable
str[len] = str[i];
str[i] = str[len-i-1];
str[len-i-1] = str[len];
}

//Assign null character back
str[len] = '\0';
}

void PrintDisplay()
{
  char *newline;
  char *oldline;
  int row = 0, sameLineFlag = 0;
  char singleLine[20];
  static int doOnce = 1;

  if(doOnce)
    {
      currentPos = Book1;
      memset(pageBreak, NULL, sizeof(pageBreak)/sizeof(char *));
      LOG_INFO("size = %d\r\n", sizeof(pageBreak)/sizeof(char *));
      pageBreak[pageCount] = Book1;
    }

  newline = strchr(currentPos, '\n');
  oldline = currentPos;

    while(newline != NULL)
      {
        memset(singleLine, '\0', sizeof(singleLine)/sizeof(char));

        if(newline - oldline > DISPLAY_ROW_LEN - 2)
          {
            newline = oldline + DISPLAY_ROW_LEN - 2;
            sameLineFlag = 1;
          }

        strncpy(singleLine, currentPos + (oldline - currentPos), newline - oldline);
        //LOG_INFO("singleLine = %s\r\n", singleLine);
        displayPrintf(row++, singleLine);

        if(row == DISPLAY_NUMBER_OF_ROWS - 1)
          {
            if(doOnce)
              {
                doOnce = 0;
                pageBreak[1] = newline;
                break;
              }

            else
              {
                pageBreak[++pageCount] = newline;
                pageCount--;
              }
            break;
          }
        if(sameLineFlag)
          {
            oldline = newline;
            sameLineFlag = 0;
          }

        else oldline = newline + 1;

        newline = strchr(currentPos + (oldline - currentPos), '\n');
      }

    LOG_INFO("pageCount = %d\r\n", pageCount);

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

void scrollDown()
{
  char *oldline;
  char *newline;
  int lineCount = 0, sameLineFlag = 0;
  int byteDiff = 0;

  char *cpystring = malloc(sizeof(char)*(currentPos - Book1 + 2));
  if(cpystring == NULL)
    return;

  strncpy(cpystring, Book1, (currentPos - Book1 + 1));
  cpystring[currentPos - Book1 + 2] = '\0';

  revString(cpystring);
  oldline = cpystring;
  newline = strchr(cpystring, '\n');

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

  byteDiff = (newline + 1) - cpystring;
  if(byteDiff < 0 || byteDiff > (int)strlen(cpystring))
    currentPos = Book1;

  else currentPos -= byteDiff;
  free(cpystring);

}

void nextPage()
{
  if(pageBreak[pageCount + 1] != NULL)
    {
      pageCount++;
      currentPos = pageBreak[pageCount];

      for(int i = 0; i < DISPLAY_NUMBER_OF_ROWS - 1; i++)
        displayPrintf(i, " ");
    }
}

void prevPage()
{
  if(pageBreak[pageCount - 1] != NULL)
    {
      pageCount--;
      currentPos = pageBreak[pageCount];

      for(int i = 0; i < DISPLAY_NUMBER_OF_ROWS - 1; i++)
        displayPrintf(i, " ");
    }
}
