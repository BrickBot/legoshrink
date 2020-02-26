///////////////////////////////////////////////////////////////////////////////
//
// File: legoshrink.c
//
// Author: Mike LeSauvage (based on work done by those in the description.)
//
// Description: This program is used to collect information from a Lego
//              robot, print it to the screen, and write it to a log (if
//              desired).
//              This program is derived from ir3, a program written by
//              Pavel Petrovic(ppetrovic@acm.org).  He in turn based it on
//              firmdl3.c and ir.c, the firmware dowloader for the RCX.  His
//              code was tested on brickOS 0.2.6.10.6 and Debian Linux.
//              (http://www.idi.ntnu.no/~petrovic/ir/)
//
//              I modified his program for my purposes, which at the moment,
//              only involve reading messages from the brick.  I documented
//              work that was already implemented, researched and documented
//              the Lego Network Protocol (see lnp_integrity_byte), removed
//              the functions related to sending data, changed the program
//              options, and added address packet support (which was the main
//              reason I started poking around in the first place).
//
//              This program was tested on brickOS 0.2.6.10 on Windows 2000.
//              Since I do not have a serial tower, I don't know if the
//              program will function with one.  I tried to preserve serial
//              related code.  Your mileage may vary.
//
//              As this is code derived from code based firmdl3, the Mozilla
//              Public License still applies.  The original firmdl3.c licence
//              is included below for your viewing pleasure.
//
// Modifications: 10 Feb 2005 - Major revision of original code complete.
//                15 Feb 2005 - Program may now be compiled in either
//                              English or French.
//                12 Mar 2005 - Changed program so it can send a file, line
//                              by line.
//
///////////////////////////////////////////////////////////////////////////////

   /*  -----------------------------------------begin-firmdl3.c-licence-----
    *  firmdl3.c
    *
    *  A firmware downloader for the RCX.  Version 3.0.  Supports single and
    *  quad speed downloading.
    *
    *  The contents of this file are subject to the Mozilla Public License
    *  Version 1.0 (the "License"); you may not use this file except in
    *  compliance with the License. You may obtain a copy of the License at
    *  http://www.mozilla.org/MPL/
    *
    *  Software distributed under the License is distributed on an "AS IS"
    *  basis, WITHOUT WARRANTY OF ANY KIND, either express or implied. See the
    *  License for the specific language governing rights and limitations
    *  under the License.
    *
    *  The Original Code is Firmdl code, released October 3, 1998.
    *
    *  The Initial Developer of the Original Code is Kekoa Proudfoot.
    *  Portions created by Kekoa Proudfoot are Copyright (C) 1998, 1999
    *  Kekoa Proudfoot. All Rights Reserved.
    *
    *  Contributor(s): Kekoa Proudfoot <kekoa@graphics.stanford.edu>
    *                  Laurent Demailly
    *                  Allen Martin
    *                  Markus Noga
    *                  Gavin Smyth
    *                  Luis Villa
    *
    *  ------------------------------------------end-firmdl3.c-licence-----
    *  modifications for ir3: Pavel Petrovic, ppetrovic@acm.org
    */

//#define FRENCH 1
#ifdef FRENCH
  #define STR_RAW              "\nMessage de données non traité:\n"
  #define STR_INTEGRITY        "\n(Intégrité)  %s\n"
  #define STR_VERBOSE          "\nHôte de dest: %d, Port de dest: %d, Hôte de srce: %d, Port de srce: %d\n%s\n"
  #define STR_UNKNOWN          "\n(Type inconnu) %s\n"
  #define STR_BAD_CHECKSUM     "\n  *** Mais Checksum!  Message(s) perdu(s)! ***\n"
  #define STR_SIZE_MISMATCH    "\n  *** Message Size Incorrect!  Message(s) Lost! ***\n"
  #define STR_OPEN_ERR         "Ne pouvait pas ouvrir %s. Le programme termine."
  #define STR_WRITE_ERR        "Ne pouvait pas écrire à %s. Le programme termine."
  #define STR_CLOSE_ERR        "Ne pouvait pas fermer %s. Le programme termine."
  #define STR_ACCESS_ERR       "Erreur dans l'acces %s.  Le programme termine."
  #define STR_HOST_ADD_RANGE   "L'adresse de l'hôte est hors limite %s."
  #define STR_NO_PARAMETER     "Paramètre manquant pour l'option %s."
  #define STR_HOST_PORT_RANGE  "Le port de l'hôte est hors limite %s."
  #define STR_UNRECOGNIZED_OPT "Option inconnue: %s."
  #define STR_FILE_NOT_DELETED "\nInfo de déboggage: Fichier non effacé.\n"
  #define STR_DEFAULT_TTY      "Utilise le tty par défaut connexion peut faillir! Utilisez -h pour les options.\n"
#else
  #define STR_RAW              "\nRaw message data:\n"
  #define STR_INTEGRITY        "\n(Integrity)  %s\n"
  #define STR_VERBOSE          "\nDest Host: %d, Dest Port: %d, Source Host: %d, Source Port: %d\n%s\n"
  #define STR_UNKNOWN          "\n(Unknown Type) %s\n"
  #define STR_BAD_CHECKSUM     "\n  *** Bad Checksum!  Message(s) Lost! ***\n"
  #define STR_SIZE_MISMATCH    "\n  *** Message Size Incorrect!  Message(s) Lost! ***\n"
  #define STR_OPEN_ERR         "Could not open %s.  Program will exit."
  #define STR_WRITE_ERR        "Could not write to %s.  Program will exit."
  #define STR_CLOSE_ERR        "Could not close %s.  Program will exit."
  #define STR_ACCESS_ERR       "Error in accessing %s.  Program will exit."
  #define STR_HOST_ADD_RANGE   "Host address is out of range %s."
  #define STR_NO_PARAMETER     "Parameter missing for %s option."
  #define STR_HOST_PORT_RANGE  "Host port is out of range %s."
  #define STR_UNRECOGNIZED_OPT "Unrecognized option: %s"
  #define STR_FILE_NOT_DELETED "\nDebug Info: File not deleted.\n"
  #define STR_DEFAULT_TTY      "Using default tty...may fail to connect!  Use -h for options.\n"
#endif


#if defined(_WIN32)
  #include <windows.h>
#endif


#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>
#include <termios.h>
#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include <ctype.h>
#include <string.h>
#include <assert.h>
#include <time.h>
#include "rcx_comm.h"

//State of the LNP protocol when receiving a packet.
typedef enum { LNPwaitHeader, LNPwaitLength,
               LNPwaitDest,   LNPwaitHost,
               LNPwaitData,   LNPwaitCRC    } lnp_integrity_state_t;
               
//Used to return the type of packet received.
typedef enum { msg_BAD_CHECKSUM, msg_PROCESSING,
               msg_INTEGRITY,    msg_ADDRESSED,
               msg_UNKNOWN,      msg_SIZE_MISMATCH } MSG_TYPES;


//Global variables
int tty_usb = 0;                  //0 if using serial port, 1 for usb.
extern int __comm_debug;
static char packet[257];          //Mike notes: Packet has max 255 bytes
                                 // data, then add 1 for null char, so why 257?
static int destHost=0, destPort=0;      //Globals to record the packet's source
static int sourceHost=0, sourcePort=0; //and destination addresses.
static char *progname;
static int showDebugInfo=0;


//Defines
#define BUFFERSIZE      4000
#define WAKEUP_TIMEOUT  4000
#define DEFAULTTTY      "/dev/ttyS0" //Linux - COM1


//Prototypes
void          LogString(char *fileName, char *aPacket);
unsigned char lnp_checksum( const unsigned char *data, unsigned length);
int           lnp_integrity_byte(unsigned char b, int size);
int           ReceiveMessage(FILEDESCR fd, int myAdd, int myPort,
                             int verbose, char *aFile, char *match);
void          SendFile(FILEDESCR fd, char *sendFile, char *rtrMsg, int verbose);


///////////////////////////////////////////////////////////////////////////////
//
// Function: lnp_checksum
//
// Description: Original docs for this function stated "modified originally
//              LegOS sources.  It calculates a checksum based on the entire
//              contents of the incoming packet (except, of course, the last
//              byte holding the checksum itself).
//
// Parameters: data - A pointer to the packet.
//             length - The number of bytes contained in the packet.
//
// Returns: unsigned char - The checksum.
//
///////////////////////////////////////////////////////////////////////////////
unsigned char lnp_checksum( const unsigned char *data, unsigned length)
{
  unsigned char a = 0xff;
  while (length > 0) {
    a = a + *data;
    data++;
    length--;
  }
  return a;
}


///////////////////////////////////////////////////////////////////////////////
//
// Function: lnp_integrity_byte
//
// Description: The previous documentation stated that this is
//              "LNP, bit modified from LegOS sources".  From examination, this
//              code is used to examine an incoming message, byte by byte and
//              track the "state" of the packet until it is fully processed.
//              Since only one byte is examined at a time, static variables
//              are used to remember state.
//
// Modifications: Mike LeSauvage(ML) changed this function to identify
//                addressed packets, properly copy their values, and return
//                different codes for different packet types.
//
// Notes: LNP packets are 1 of the 2 following formats (all I've seen so far)
// 
//    INTEGRITY: header|length|destadd|sourceadd|up to 253 bytes data|checksum
//
//    ADDRESSED: header|length|up to 255 bytes data|checksum
//
//     Each section is one byte long, so a message at max data is 258 bytes.
//
//      header: Format is 11110xxx (base 2), where xxx may be changed to allow
//              different header types.  The only types identified by ML are
//              the integrity packet(000) and an addressed packet(001).
//
//      length: The length in bytes of the data and address sections combined.
//
//     destadd: The destination address in the format hhhhpppp where hhhh is
//              the host and pppp is the port for that host. (base 2)
//
//   sourceadd: This field follows the same format as destadd.
//
//    checksum: A value calculated by the sending protocol.  It should conform
//              to the lnp_checksum() function in this file.
//
//
// Parameters: b    - A single byte for inspection.
//             size - The number of bytes total in the packet
//
// Returns: int: Use a value from MSG_TYPE: msg_BAD_CHECKSUM
//                                          msg_PROCESSING
//                                          msg_INTEGRITY
//                                          msg_ADDRESSED
//                                          msg_UNKNOWN
//
///////////////////////////////////////////////////////////////////////////////
int lnp_integrity_byte(unsigned char b, int size)
{
  static unsigned char buffer[259];
  static int bytesRead,endOfData;
  static int lnp_integrity_state = LNPwaitHeader;
  static unsigned char packetType; //Holds the non-masked portion of header
  
  if(lnp_integrity_state==LNPwaitHeader)
    bytesRead=0;

  buffer[bytesRead++]=b;


  switch(lnp_integrity_state)
  {
    case LNPwaitHeader:                                     //Check top five
      if((b & (unsigned char) 0xf8) == (unsigned char) 0xf0)//bits="11110"
      {
        packetType = b & (unsigned char)0x7;        //Extract the packet type
        lnp_integrity_state=LNPwaitLength;          //from the header
        break;                                      //(bottom three bits)
      }

    case LNPwaitLength:
      endOfData=b+2;
      if(endOfData+1 != size)                //size is number of bytes rec'd
      {                                      //and it should match what the
        lnp_integrity_state=LNPwaitHeader;   //length byte states (+3 because
        return msg_SIZE_MISMATCH;            //that byte doesn't include the
      }                                      //header, size, and checksum.)
      if(packetType==1)                      //If this is an addressed packet
        lnp_integrity_state=LNPwaitDest;    //then wait on the destination byte
      else                                 //otherwise, go on to receive data.
        lnp_integrity_state=LNPwaitData;
      break;
        
    case LNPwaitDest:                       //Record the host and port to
      destHost=b>>4;                       //which this message was addressed.
      destPort=b&0xf;
      lnp_integrity_state=LNPwaitHost;
      break;

    case LNPwaitHost:                      //Record the source host and port.
      sourceHost=b>>4;
      sourcePort=b&0xf;
      lnp_integrity_state=LNPwaitData;
      break;

    case LNPwaitData:
      if(bytesRead==endOfData)
        lnp_integrity_state=LNPwaitCRC;
      break;

    case LNPwaitCRC:
      if(b == (unsigned char)lnp_checksum(buffer,endOfData))
      {
         buffer[buffer[1]+2]=0;            //Put terminating null at string end
         if(packetType==1)                   //Copy position from the buffer is
         {                                   //4 bytes in for an addressed
           strcpy(packet, buffer+4);         //packet.  Reset integrity state
           lnp_integrity_state=LNPwaitHeader;//and return the addressed msg
           return msg_ADDRESSED;             //flag.
         }
         else if(packetType==0)              //Copy position from the buffer is
         {                                   //2 bytes for an integrity packet.
           strcpy(packet,buffer+2);
           lnp_integrity_state=LNPwaitHeader;//Reset integrity state and return
           return msg_INTEGRITY;             //the integrity msg flag.
         }
         else                                //Unknown packet type.  Make a
         {                                   //guess as to buffer contents and
           strcpy(packet,buffer+2);          //return the unknown flag.
           lnp_integrity_state=LNPwaitHeader;
           return msg_UNKNOWN;
         }
      }
      else                                      //Bad checksum.
      {
        lnp_integrity_state=LNPwaitHeader;
        return msg_BAD_CHECKSUM;
      }
    }
  return msg_PROCESSING;
}


///////////////////////////////////////////////////////////////////////////////
//
// Function: ReceiveMessage
//
// Description: Original documentation for this function stated
//              "receive next LNP message to global array packet[]"
//              This function receives messages from the USB tower and
//              prints them to the screen until the program is exited.
//              Originally, this function sat in a loop receiving messages
//              until the program was closed.  It has been changed to
//              receive a single message.
//
// Parameters: fd - A file descriptor for the USB tower.
//             myAdd - Address of this PC; ignore packets with wrong dest add.
//             myPort - Port of this PC; ignore packets with wrong dest port.
//             verbose - Provide legend for incoming address info.
//             aFile - Name of a file to log the data.  NULL if not logging.
//             match - A pointer to a string that should be compared to the
//                     received message.  Pass NULL for no comparison.
//
// Returns: int - 1 if the received packet matches the provided string
//                "match", false otherwise.
//
///////////////////////////////////////////////////////////////////////////////
int ReceiveMessage(FILEDESCR fd, int myAdd,   int myPort,
                                 int verbose, char *aFile, char *match)
{
  int status, len, i, j=0, messageStatus;
  unsigned char recv[BUFFERSIZE + 1];
  unsigned char outputString[256 + 80]="";        //256 for packet +
                                                  //terminating null.  80 for
                                                  //extra info ie: address
                                                  //static as this fn will
                                                  //be called frequently.
                                           
  int receive_timeout=100;          //No idea how this timeout actually works.
                                    //Changing the value doesn't seem to
                                    //make any difference.

  if(tty_usb==0)                                              //Wake up the
  {                                                           //serial tower.
    if ((status = rcx_wakeup_tower(fd, WAKEUP_TIMEOUT)) < 0)  //Return if it
    {                                                         //can't be woken. 
      fprintf(stderr, "%s: %s\n", progname, rcx_strerror(status));
      return 0;
    }
  }

  packet[0] = 0;               //Ensure packet starts with terminating null?
  len = nbread(fd, recv, BUFFERSIZE, receive_timeout); //Read in a message.

  i = 0; j++; messageStatus=msg_PROCESSING;
  while(i<len  && messageStatus==msg_PROCESSING) //Process messages through
  {                                              //LNP one byte at a time...
    if(showDebugInfo)
    {
      if(i==0)                                   //This section prints out
        printf(STR_RAW);                         //all the raw message bytes
      if(i%16 !=15)                              //if the user requested
      {                                          //debug information.
        if(i==len-1)                             //Info is organized on
          printf("%3d\n", recv[i]);              //3-digit bytes (in decimal)
        else                                     //with 16 bytes per line.
          printf("%3d, ", recv[i]);
      }
      else
        printf("%3d\n", recv[i]);
    }
    messageStatus=lnp_integrity_byte(recv[i], len);
    i++;
  }

  switch(messageStatus)
  {                                           //Message has been received,
    case msg_INTEGRITY:                       //and the output may now be
      sprintf(outputString,                   //generated.  Ensure that the
              STR_INTEGRITY, packet);         //destination host and port are
      break;                                  //this machine (or that the
    case msg_ADDRESSED:                       //user asked for messages to
      if(destHost==myAdd)                     //any port).  Format based on
        if(destPort==myPort || myPort==-1)    //the verbose option.
        {
          if(verbose)
          {
            sprintf(outputString, STR_VERBOSE, destHost, destPort,
                                               sourceHost, sourcePort,
                                               packet);
          }
          else
          {
            sprintf(outputString, "\n(%d,%d|%d,%d)  %s\n", destHost,
                                                           destPort,
                                                           sourceHost,
                                                           sourcePort,
                                                           packet);
          }
        }
      break;
    case msg_UNKNOWN:
      sprintf(outputString, STR_UNKNOWN, packet);
      break;
    case msg_BAD_CHECKSUM:
      sprintf(outputString, STR_BAD_CHECKSUM);
      break;
    case msg_SIZE_MISMATCH:
      sprintf(outputString, STR_SIZE_MISMATCH);
      break;
    default:
      outputString[0]='\0';
  }
  
  if(strlen(outputString) > 0)
  {
    printf("%s", outputString);
    if(aFile != NULL)
      LogString(aFile, outputString);
  }
  

  if (tty_usb == 0)                        //This appears to be some
  {                                        //code related to the serial
    if (j * receive_timeout > 3000)        //tower.  It executes every
    {                                      //30 messages as is.  My
      j = 0;                               //guess is it has something
      recv[0] = 0x10;                      //to do with serial tower sleep
      mywrite(fd, recv, 1);                //but I don't really know so
    }                                      //I'm just leaving it alone.
  }
#ifdef _WIN32
  FlushFileBuffers(fd);
#endif

  if(match != NULL)
    if(!strcmp(packet, match))
      return 1;
  return 0;
}


///////////////////////////////////////////////////////////////////////////////
//
// Function: SendFile
//
// Description: This function sends a file over the USB tower, one line of text
//              at a time.  There is a limit of 255 characters per line, any
//              excess will be truncated.  Only integrity communication is
//              currently supported.
//              The robot must send a message requesting each line.
//
// Parameters: fd - A file descriptor for the USB tower.
//             sendFile - The path/name of the file to send.
//             rtrMsg - A string to be matched.  Before sending a line, this
//                      waits to receive a match to this string.  When it does,
//                      the next line in the file is sent.
//             verbose - Does nothing in this function; is used to tell the
//                       ReceiveMessage function if verbose reporting should
//                       be used.
//
// Returns: Nothing.
//
///////////////////////////////////////////////////////////////////////////////
void SendFile(FILEDESCR fd, char *sendFile, char *rtrMsg, int verbose)
{
  int i;
  long fileLen;
  int msgLen;
  int fileHandle;
  int ncRead;
  int curPos=0;
  char *fileBuffer;
  char msgBuf[258]; //255 char message, 1 char header,
                    //1 char length, 1 char checksum.

  fileHandle=open(sendFile, O_RDONLY|O_BINARY);  //Try to open the file.
  if(fileHandle == -1)
  {
    fprintf(stderr, STR_OPEN_ERR, sendFile);
    exit(1);
  }

  fileLen=lseek(fileHandle, 0, SEEK_END);     //Determine the size of the file.
  if(fileLen == -1L)
  {
    fprintf(stderr, STR_ACCESS_ERR, sendFile);
    exit(1);
  }

  lseek(fileHandle, 0, SEEK_SET);                //Seek back to start.
  fileBuffer = (char *)malloc(fileLen + 1);      //Allocate enough space for
                                                 //the file plus an end null.

  ncRead = read(fileHandle, fileBuffer, fileLen);//Read in the file and
  if(ncRead != fileLen)                          //check to see if the entire
  {                                              //thing was read in.  If not,
    fprintf(stderr, STR_ACCESS_ERR, sendFile);   //an error has ocurred.
    exit(1);
  }
  fileBuffer[fileLen]=0;                         //Set an end null to be sure.

  close(fileHandle);

  for(i=0; i<ncRead; i++)                        //Replace all carriage
    if(fileBuffer[i]==0xD || fileBuffer[i]==0xA) //returns with null
      fileBuffer[i]=0;                           //characters.

  while(curPos < fileLen)
  {
    while(!ReceiveMessage(fd, 0, 0, verbose, NULL, rtrMsg));//Wait for the rtr.

    msgLen=strlen(&fileBuffer[curPos]);            //Find length to next null.
    if(msgLen > 255)                               //If more than 255, set 255.
      msgLen = 255;
    for(i=0; i<msgLen; i++)                 //Now copy the line to the buffer.
      msgBuf[i+2]=fileBuffer[curPos+i];     //Leave space for header & length!

    msgBuf[0]=0xF0;                                          //Set header.
    msgBuf[1]=msgLen;                                        //Set packet size.
    msgBuf[msgLen+2]=(char)lnp_checksum(msgBuf, msgLen + 2); //Set checksum.
    mywrite(fd, msgBuf, msgLen + 3);                         //Send to usb twr.

    printf("Sent: ");
    if(showDebugInfo)
      for(i=0; i<msgLen+3; i++)
        printf("%c(%d) ", msgBuf[i], msgBuf[i]);
    else
      for(i=2; i<msgLen+2; i++)
        printf("%c", msgBuf[i]);
    printf("\n");

    while(fileBuffer[curPos] != 0)      //Find next null character.
      curPos++;                         //There could me multiple nulls, so
    while(fileBuffer[curPos] == 0)      //increment to next real character.
      curPos++;
  }

  free(fileBuffer);
  printf("\n\nFile %s has been completely sent.\n", sendFile);
  
}


///////////////////////////////////////////////////////////////////////////////
//
// Function: LogString
//
// Description: This function writes a string to the specified file.  If it
//              fails to open, close, or write to the file it exits the
//              program.
//
// Parameters: fn      - The file in which to write the string.
//             aString - The string to write.
//
// Returns: Nothing.
//
///////////////////////////////////////////////////////////////////////////////
void LogString(char *fn, char *aString)
{
  int fileHandle;
  int bytesWritten;
  
  fileHandle=open(fn, O_WRONLY|O_CREAT|O_APPEND|O_TEXT, S_IREAD|S_IWRITE);
    
  if(fileHandle==-1)
  {
    fprintf(stderr, STR_OPEN_ERR, fn);
    exit(1);
  }
  
  bytesWritten = write(fileHandle, aString, strlen(aString));
  if(bytesWritten!=strlen(aString))
  {
    fprintf(stderr, STR_WRITE_ERR, fn);
    close(fileHandle);
    exit(1);
  }
    
  if(close(fileHandle)==-1)
  {
    fprintf(stderr, STR_CLOSE_ERR, fn);
    exit(1);
  }
  
  return;
}  


///////////////////////////////////////////////////////////////////////////////
//
// Function: main
//
// Description: The main function for the legoshrink program.  The purpose
//              of main is to parse the command line arguments and set up the
//              appropriate environment for receiving packets from the lego
//              robot.
//
// Parameters: argc - The number of command line arguments.
//             argv - An array of pointers to strings of the command line args.
//
// Returns: 0 for normal exit,
//          1 on abnormal termination.
//
///////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{
  FILEDESCR fd;
  int addressPC=0;
  int portPC=-1;
  int verbose=0;
  int useFast=0;
  int overWrite=0;
  char errorMsg[60]="";
  char *logFileName=NULL;
  char *sendFileName=NULL;
  char *rtrWord=NULL;
  char *tty=NULL;

#ifdef FRENCH
  char *usageString =
      "Utilisation: legoshrink [options]\n\n"
      "Options:\n"
      "  -a <adresse>     Règle l'adresse du PC hôte, valide de 0-15.  Défaut 0.\n"
      "  -p <port>        Règle le port du PC hôte pour écouter, valide de 0-15.\n"
      "                   Utilise -1 pour écouter pour des messages dirigées à\n"
      "                   n'importe quel port (default).\n"
      "  -l <fichier>     Enregistre tout les messages au fichier spécifique.\n"
      "  -ld <fichier>    Enregistre tout les messages au fichier; efface le fichier\n"
      "                   si il existe déjà.\n"
      "  -sf <file> <rtr> Envoie fichier. Fichier envoyé ligne par ligne (caractères\n"
      "                   excédent 255 sont enlevés).  Avant d'envoyer chaque \n"
      "                   ligne, legoshrink attend de recevoir le mot par (prêt à \n"
      "                   recevoir) du robot.  Seul le mote intégrité est supporté.\n"
      "  -t <connexion>   Spécifie le type de connexion (TTY ou usb).\n"
      "  -f               Communique en mode rapide (sériel seulement).\n"
      "  -v               Verbose (Plus d'information sur les messages adressés)\n"
      "  -h               Affiche cet aide et sort\n"
      "  --déboguage      Montre les octets non-traités pour tout les messages\n"
      "                   qui arrivent.\n"
      "\n"
      "Exemples: legoshrink -t usb -p 6 -l crashlog.txt\n"
      "          legoshrink -t usb -sf directives.txt ENVOIE_TDS\n"
      ;
#else
  char *usageString =
      "Usage: legoshrink [options]\n\n"
      "Options:\n"
      "  -a <address>     Set PC host address, valid from 0-15.  Default 0.\n"
      "  -p <port>        Set PC host port to listen on, valid from 0-15.\n"
      "                   Use -1 to listen for messages directed to any port(default).\n"
      "  -l <file>        Log all messages to the specified file.\n"
      "  -ld <file>       Log messages; first delete named file if it already exists.\n"
      "  -sf <file> <rtr> Send file.  The file is sent one line at a time (characters\n"
      "                   in excess of 255 will be truncated).  Before sending each\n"
      "                   line, legoshrink waits to receive the rtr word (ready to\n"
      "                   receive) from the bot.  Only integrity mode is supported.\n"
      "  -t <connection>  Specify connection type (TTY or usb).\n"
      "  -f               Communicate in fast mode (serial only).\n"
      "  -v               Verbose (More information on addressed messages)\n"
      "  -h               Display this help and exit\n"
      "  --debug          Show raw bytes for all incoming messages.\n"
      "\n"
      "Examples: legoshrink -t usb -p 6 -l crashlog.txt\n"
      "          legoshrink -t usb -sf commands.txt SENDNOW\n"
      ;
#endif
   
      
  //Parse the command line.
  progname=argv[0];
  argc--;
  argv++;

  while(argc > 0 && strlen(errorMsg)==0)
  {
    if(!strcmp(argv[0], "--debug"))            //Extract debug option.
      showDebugInfo = 1;
    else if(!strcmp(argv[0], "-a"))            //Extract host address option.
    {
      if(argc > 1)
      {
        argc--;
        argv++;
        addressPC=atoi(argv[0]);
        if(addressPC<0 || addressPC>15)
          sprintf(errorMsg, STR_HOST_ADD_RANGE, "0-15");
      }
      else
        sprintf(errorMsg, STR_NO_PARAMETER, "-a");
    }
    else if(!strcmp(argv[0], "-p"))            //Extract host port setting.
    {
      if(argc > 1)
      {
        argc--;
        argv++;
        portPC=atoi(argv[0]);
        if(portPC<-1 || portPC>15)
          sprintf(errorMsg, STR_HOST_PORT_RANGE, "(-1)-15");
      }
      else
        sprintf(errorMsg, STR_NO_PARAMETER, "-p");
    }
    else if(!strcmp(argv[0], "-l"))           //Extract file option.
    {
      if(argc > 1)
      {
        argc--;
        argv++;
        logFileName=argv[0];
      }
      else
        sprintf(errorMsg, STR_NO_PARAMETER, "-l");
    }
    else if(!strcmp(argv[0], "-ld"))          //Extract file option.
    {                                         //along with the
      if(argc > 1)                            //overwrite option.
      {
        argc--;
        argv++;
        logFileName=argv[0];
        overWrite=1;
      }
      else
        sprintf(errorMsg, STR_NO_PARAMETER, "-ld");
    }
    else if(!strcmp(argv[0], "-sf"))        //Extract send file option
    {                                       //along with its two parameters:
      if(argc > 2)                          //the file to send and the
      {                                     //"ready to receive" word.
        argc--;
        argv++;
        sendFileName=argv[0];
        argc--;
        argv++;
        rtrWord=argv[0];
      }
      else
        sprintf(errorMsg, STR_NO_PARAMETER, "-sf");
    }
    else if(!strcmp(argv[0], "-t"))           //Extract tty option.
    {
      if(argc > 1)
      {
        argc--;
        argv++;
        tty = argv[0];
      }
      else
        sprintf(errorMsg, STR_NO_PARAMETER, "-t");
    }
    else if(!strcmp(argv[0], "-f"))            //Extract fast option.
      useFast=1;
    else if(!strcmp(argv[0], "-v"))            //Extract verbose option.
      verbose=1;
    else if(!strcmp(argv[0], "-h"))            //Extract help option.
      sprintf(errorMsg, " ");
    else
      sprintf(errorMsg, STR_UNRECOGNIZED_OPT, argv[0]);

    argc--;
    argv++;
  }

  if(strlen(errorMsg)>0)                    //Produce error messages
  {                                         //(if any), display usage
    fprintf(stderr, "\n%s\n", errorMsg);    //information, and quit.
    fprintf(stderr, "\n%s", usageString);
    exit(1);
  }

  if(logFileName != NULL && overWrite==1)   //If the user specified overWrite
  {                                         //then delete the log file if
    int err;                                //one exists.
    err=remove(logFileName);
    if(err==-1 && showDebugInfo)
      fprintf(stderr, STR_FILE_NOT_DELETED);
  }

  if (!tty)                       //Get the tty name.
  {
    tty = getenv("RCXTTY");
    printf(STR_DEFAULT_TTY);
  }
  if (!tty)                       //No environment var set, use default.
    tty = DEFAULTTTY;

  if(strcmp(tty,"usb")==0)        //Check if usb was chosen.
  {
    tty_usb = 1;                  //Mark it as the communications method.
    tty = "\\\\.\\legotower1";    //This is the default tower. Systems with
                                  //more than one would have to change this.
  }

  if (useFast && (tty_usb==0))
    fd = rcx_init(tty, 1);        //Only serial has a fast mode.
  else
    fd = rcx_init(tty, 0);        //Either fast not selected or USB in use.



  if(sendFileName == NULL)
  {
    while(1)
      ReceiveMessage(fd, addressPC, portPC, verbose, logFileName, NULL);
  }
  else
  {
    SendFile(fd, sendFileName, rtrWord, verbose);
  }

  rcx_close(fd);
  exit(0);
}

