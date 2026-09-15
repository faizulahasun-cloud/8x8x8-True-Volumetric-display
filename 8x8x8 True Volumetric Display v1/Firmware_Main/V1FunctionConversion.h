#pragma once
#include <Arduino.h>
#include <avr/pgmspace.h>
#include <math.h>

// =============================================================================
// UNIVERSE ENGINE: FIXED-POINT 8.8 CONFIGURATION
// =============================================================================
typedef int16_t fixed;
#define TO_FIXED(x) ((fixed)((x) * 256.0))
#define INT_FROM_FIXED(x) ((x) >> 8)
#define FX_ONE 256

// Fast Sine Look-Up Table (LUT) in Flash
const int16_t sinTable[256] PROGMEM = {
    0, 6, 12, 19, 25, 31, 37, 44, 50, 56, 62, 68, 74, 80, 86, 92,
    98, 103, 109, 115, 120, 126, 131, 136, 142, 147, 152, 157, 162, 167, 171, 176,
    180, 185, 189, 193, 197, 201, 205, 208, 212, 215, 219, 222, 225, 228, 231, 233,
    236, 238, 240, 242, 244, 246, 247, 249, 250, 251, 252, 253, 254, 254, 255, 255,
    255, 255, 255, 254, 254, 253, 252, 251, 250, 249, 247, 246, 244, 242, 240, 238,
    236, 233, 231, 228, 225, 222, 219, 215, 212, 208, 205, 201, 197, 193, 189, 185,
    180, 176, 171, 167, 162, 157, 152, 147, 142, 136, 131, 126, 120, 115, 109, 103,
    98, 92, 86, 80, 74, 68, 62, 56, 50, 44, 37, 31, 25, 19, 12, 6,
    0, -6, -12, -19, -25, -31, -37, -44, -50, -56, -62, -68, -74, -80, -86, -92,
    -98, -103, -109, -115, -120, -126, -131, -136, -142, -147, -152, -157, -162, -167, -171, -176,
    -180, -185, -189, -193, -197, -201, -205, -208, -212, -215, -219, -222, -225, -228, -231, -233,
    -236, -238, -240, -242, -244, -246, -247, -249, -250, -251, -252, -253, -254, -254, -255, -255,
    -255, -255, -255, -254, -254, -253, -252, -251, -250, -249, -247, -246, -244, -242, -240, -238,
    -236, -233, -231, -228, -225, -222, -219, -215, -212, -208, -205, -201, -197, -193, -189, -185,
    -180, -176, -171, -167, -162, -157, -152, -147, -142, -136, -131, -126, -120, -115, -109, -103,
    -98, -92, -86, -80, -74, -68, -62, -56, -50, -44, -37, -31, -25, -19, -12, -6
};

namespace V1FunctionConversion {

static const uint16_t MAX_FUNCTION_LENGTH = 256;
static const uint8_t MAX_BYTECODE_LENGTH = 150;

enum OpCode:uint8_t{
    OP_END=0, OP_CONST, OP_X, OP_Y, OP_Z, OP_F, 
    OP_ADD, OP_SUB, OP_MUL, OP_DIV, OP_MOD, OP_NEG, 
    OP_SIN, OP_COS, OP_SQRT, OP_ABS, OP_NOT, 
    OP_LT, OP_LE, OP_GT, OP_GE, OP_EQ, OP_NE, 
    OP_AND, OP_OR, OP_MAX, OP_MIN, OP_DIST, OP_SQR
};

struct Instruction{uint8_t op;fixed value;};

static char functionBuffer[MAX_FUNCTION_LENGTH+1];
static uint16_t functionLength=0;
static bool functionStarted=false;
static bool functionComplete=false;
static bool functionValid=false;
static bool receiveError=false;
static Instruction bytecode[MAX_BYTECODE_LENGTH];
static uint8_t bytecodeLength=0;
static uint16_t parsePosition=0;
static bool parseError=false;

// Fixed-Point Math Primitives
inline fixed fx_sin(uint8_t angle) { return (fixed)pgm_read_word(&sinTable[angle]); }
inline fixed fx_cos(uint8_t angle) { return (fixed)pgm_read_word(&sinTable[(uint8_t)(angle + 64)]); }

inline fixed fx_sqrt(fixed val) {
    if (val <= 0) return 0;
    // To get root in 8.8 format, input must be in 16.16 format
    int32_t val32 = (int32_t)val << 8;
    int32_t res = val32;
    int32_t prev;
    do {
        prev = res;
        res = (res + val32 / res) >> 1;
    } while (abs(res - prev) > 1);
    return (fixed)res;
}

inline void resetReception(){
  functionLength=0;functionStarted=true;functionComplete=false;functionValid=false;receiveError=false;parsePosition=0;parseError=false;
  functionBuffer[0]='\0';
}
inline void startReception(){resetReception();}
inline void cancelReception(){
  functionStarted=false;functionComplete=false;functionValid=false;
}
inline void stopReception(){
  if(!functionStarted)return;
  functionBuffer[functionLength]='\0';
  functionComplete=(functionLength>0&&!receiveError);
  functionStarted=false;
}
inline bool receiveCharacter(char c){
  if(!functionStarted)return false;
  if(c=='\r'||c=='\n')return false;
  if((uint8_t)c<0x20||(uint8_t)c>0x7E){receiveError=true;return false;}
  if(functionLength>=MAX_FUNCTION_LENGTH){receiveError=true;return false;}
  functionBuffer[functionLength++]=c;functionBuffer[functionLength]='\0';
  return false;
}
inline bool isFunctionStarted(){return functionStarted;}
inline bool isFunctionComplete(){return functionComplete;}
inline bool isFunctionValid(){return functionValid;}
inline uint16_t receivedLength(){return functionLength;}
inline void skipSpaces(){while(parsePosition<functionLength&&functionBuffer[parsePosition]==' ')parsePosition++;}
inline bool matchChar(char c){skipSpaces();if(parsePosition<functionLength&&functionBuffer[parsePosition]==c){parsePosition++;return true;}return false;}
inline bool emit(uint8_t op,fixed value=0){if(bytecodeLength>=MAX_BYTECODE_LENGTH-1){parseError=true;return false;}bytecode[bytecodeLength].op=op;bytecode[bytecodeLength].value=value;bytecodeLength++;return true;}
inline bool parseExpression();

inline bool parseNumber(){
  skipSpaces();if(parsePosition>=functionLength||functionBuffer[parsePosition]<'0'||functionBuffer[parsePosition]>'9')return false;
  float valF=0.0f;
  while(parsePosition<functionLength){char c=functionBuffer[parsePosition];if(c<'0'||c>'9')break;valF=valF*10.0f+(float)(c-'0');parsePosition++;}
  if(parsePosition<functionLength&&functionBuffer[parsePosition]=='.'){
    parsePosition++;if(parsePosition>=functionLength||functionBuffer[parsePosition]<'0'||functionBuffer[parsePosition]>'9'){parseError=true;return false;}
    float place=0.1f;while(parsePosition<functionLength){char c=functionBuffer[parsePosition];if(c<'0'||c>'9')break;valF+=(float)(c-'0')*place;place*=0.1f;parsePosition++;}
  }
  return emit(OP_CONST, TO_FIXED(valF));
}
inline char upperAscii(char c){return(c>='a'&&c<='z')?(char)(c-'a'+'A'):c;}
inline bool isReservedCommandToken(char c){
  c=upperAscii(c);
  return c=='A'||c=='M'||c=='N'||c=='C'||c=='E'||c=='R';
}
inline bool parseIdentifier(){
  skipSpaces();if(parsePosition>=functionLength)return false;uint16_t start=parsePosition;
  while(parsePosition<functionLength){char c=functionBuffer[parsePosition];if(!((c>='A'&&c<='Z')||(c>='a'&&c<='z')))break;parsePosition++;}
  if(start==parsePosition)return false;uint16_t n=parsePosition-start;
  if(n==1){char q=upperAscii(functionBuffer[start]);if(q=='X')return emit(OP_X);if(q=='Y')return emit(OP_Y);if(q=='Z')return emit(OP_Z);if(q=='F')return emit(OP_F);if(isReservedCommandToken(q)){parseError=true;return false;}parseError=true;return false;}
  if(!matchChar('(')){parseError=true;return false;}
  
  char n0=n>0?upperAscii(functionBuffer[start]):0,n1=n>1?upperAscii(functionBuffer[start+1]):0,n2=n>2?upperAscii(functionBuffer[start+2]):0,n3=n>3?upperAscii(functionBuffer[start+3]):0;
  
  if(n==3&&n0=='S'&&n1=='I'&&n2=='N'){ if(!parseExpression()||!matchChar(')'))return false; return emit(OP_SIN); }
  if(n==3&&n0=='C'&&n1=='O'&&n2=='S'){ if(!parseExpression()||!matchChar(')'))return false; return emit(OP_COS); }
  if(n==4&&n0=='S'&&n1=='Q'&&n2=='R'&&n3=='T'){ if(!parseExpression()||!matchChar(')'))return false; return emit(OP_SQRT); }
  if(n==3&&n0=='A'&&n1=='B'&&n2=='S'){ if(!parseExpression()||!matchChar(')'))return false; return emit(OP_ABS); }
  if(n==4&&n0=='D'&&n1=='I'&&n2=='S'&&n3=='T'){ 
      for(int i=0; i<6; i++) {
          if(!parseExpression()) return false;
          if(i<5 && !matchChar(',')) return false;
      }
      if(!matchChar(')')) return false;
      return emit(OP_DIST);
  }
  if(n==3&&n0=='M'&&n1=='A'&&n2=='X'){ if(!parseExpression()||!matchChar(',')||!parseExpression()||!matchChar(')'))return false; return emit(OP_MAX); }
  if(n==3&&n0=='M'&&n1=='I'&&n2=='N'){ if(!parseExpression()||!matchChar(',')||!parseExpression()||!matchChar(')'))return false; return emit(OP_MIN); }
  if(n==3&&n0=='S'&&n1=='Q'&&n2=='R'){ if(!parseExpression()||!matchChar(')'))return false; return emit(OP_SQR); }
  
  parseError=true;return false;
}
inline bool parsePrimary(){skipSpaces();if(matchChar('(')){if(!parseExpression())return false;return matchChar(')');}if(parseNumber())return true;return parseIdentifier();}
inline bool parseUnary(){skipSpaces();if(matchChar('-')){if(!parseUnary())return false;return emit(OP_NEG);}if(matchChar('!')){if(!parseUnary())return false;return emit(OP_NOT);}return parsePrimary();}
inline bool parseMultiplication(){if(!parseUnary())return false;while(true){if(matchChar('*')){if(!parseUnary()||!emit(OP_MUL))return false;}else if(matchChar('/')){if(!parseUnary()||!emit(OP_DIV))return false;}else if(matchChar('%')){if(!parseUnary()||!emit(OP_MOD))return false;}else return true;}}
inline bool parseAddition(){if(!parseMultiplication())return false;while(true){if(matchChar('+')){if(!parseMultiplication()||!emit(OP_ADD))return false;}else if(matchChar('-')){if(!parseMultiplication()||!emit(OP_SUB))return false;}else return true;}}
inline bool parseComparison(){
  if(!parseAddition())return false;skipSpaces();if(parsePosition>=functionLength)return true;char a=functionBuffer[parsePosition],b=(parsePosition+1<functionLength)?functionBuffer[parsePosition+1]:'\0';uint8_t op=OP_END;
  if(a=='<'&&b=='=')op=OP_LE;else if(a=='>'&&b=='=')op=OP_GE;else if(a=='='&&b=='=')op=OP_EQ;else if(a=='!'&&b=='=')op=OP_NE;else if(a=='<')op=OP_LT;else if(a=='>')op=OP_GT;else return true;
  parsePosition+=(b=='='?2:1);if(!parseAddition()||!emit(op))return false;return true;
}
inline bool parseLogicalAnd(){if(!parseComparison())return false;while(true){skipSpaces();if(parsePosition+1<functionLength&&functionBuffer[parsePosition]=='&'&&functionBuffer[parsePosition+1]=='&'){parsePosition+=2;if(!parseComparison()||!emit(OP_AND))return false;}else return true;}}
inline bool parseExpression(){if(!parseLogicalAnd())return false;while(true){skipSpaces();if(parsePosition+1<functionLength&&functionBuffer[parsePosition]=='|'&&functionBuffer[parsePosition+1]=='|'){parsePosition+=2;if(!parseLogicalAnd()||!emit(OP_OR))return false;}else return true;}}

inline bool compileFunction(){
  functionValid=false;bytecodeLength=0;parsePosition=0;parseError=receiveError;
  if(!functionComplete||functionLength==0||receiveError)return false;
  if(!parseExpression())return false;
  skipSpaces();
  if(parsePosition!=functionLength){
    while(parsePosition < functionLength && functionBuffer[parsePosition] == ' ') parsePosition++;
    if(parsePosition != functionLength) return false;
  }
  if(!emit(OP_END)){bytecodeLength=0;return false;}
  functionValid=true;return true;
}

inline bool evaluate(uint8_t X,uint8_t Y,uint8_t Z,uint8_t F){
  if(!functionValid)return false;fixed stack[32];uint8_t sp=0;
  for(uint8_t i=0;i<bytecodeLength;i++){const Instruction& ins=bytecode[i];switch(ins.op){
    case OP_END: return sp ? (stack[sp-1] != 0) : false;
    case OP_CONST:if(sp>=32)return false;stack[sp++]=ins.value;break;
    case OP_X:if(sp>=32)return false;stack[sp++]=(fixed)X << 8;break;
    case OP_Y:if(sp>=32)return false;stack[sp++]=(fixed)Y << 8;break;
    case OP_Z:if(sp>=32)return false;stack[sp++]=(fixed)Z << 8;break;
    case OP_F:if(sp>=32)return false;stack[sp++]=(fixed)F << 8;break;
    case OP_NEG:if(!sp)return false;stack[sp-1]=-stack[sp-1];break;
    case OP_NOT:if(!sp)return false;stack[sp-1]=(stack[sp-1]==0 ? FX_ONE : 0);break;
    case OP_SIN:if(!sp)return false;stack[sp-1]=fx_sin((uint8_t)(stack[sp-1] >> 8));break;
    case OP_COS:if(!sp)return false;stack[sp-1]=fx_cos((uint8_t)(stack[sp-1] >> 8));break;
    case OP_SQRT:if(!sp)return false;stack[sp-1]=fx_sqrt(stack[sp-1]);break;
    case OP_ABS:if(!sp)return false;stack[sp-1]=abs(stack[sp-1]);break;
    case OP_ADD:{if(sp<2)return false;fixed b=stack[--sp];stack[sp-1]+=b;break;}
    case OP_SUB:{if(sp<2)return false;fixed b=stack[--sp];stack[sp-1]-=b;break;}
    case OP_MUL:{if(sp<2)return false;fixed b=stack[--sp];stack[sp-1]=((int32_t)stack[sp-1]*b)>>8;break;}
    case OP_DIV:{if(sp<2)return false;fixed b=stack[--sp];if(b==0)return false;stack[sp-1]=((int32_t)stack[sp-1]<<8)/b;break;}
    case OP_MOD:{if(sp<2)return false;fixed b=stack[--sp];if(b==0)return false;stack[sp-1]=(int32_t)stack[sp-1]%b;break;}
    case OP_LT:{if(sp<2)return false;fixed b=stack[--sp];stack[sp-1]=(stack[sp-1]<b)?FX_ONE:0;break;}
    case OP_LE:{if(sp<2)return false;fixed b=stack[--sp];stack[sp-1]=(stack[sp-1]<=b)?FX_ONE:0;break;}
    case OP_GT:{if(sp<2)return false;fixed b=stack[--sp];stack[sp-1]=(stack[sp-1]>b)?FX_ONE:0;break;}
    case OP_GE:{if(sp<2)return false;fixed b=stack[--sp];stack[sp-1]=(stack[sp-1]>=b)?FX_ONE:0;break;}
    case OP_EQ:{if(sp<2)return false;fixed b=stack[--sp];stack[sp-1]=(stack[sp-1]==b)?FX_ONE:0;break;}
    case OP_NE:{if(sp<2)return false;fixed b=stack[--sp];stack[sp-1]=(stack[sp-1]!=b)?FX_ONE:0;break;}
    case OP_AND:{if(sp<2)return false;fixed b=stack[--sp];stack[sp-1]=(stack[sp-1]!=0 && b!=0)?FX_ONE:0;break;}
    case OP_OR:{if(sp<2)return false;fixed b=stack[--sp];stack[sp-1]=(stack[sp-1]!=0 || b!=0)?FX_ONE:0;break;}
    case OP_MAX:{if(sp<2)return false;fixed b=stack[--sp];stack[sp-1]=max(stack[sp-1],b);break;}
    case OP_MIN:{if(sp<2)return false;fixed b=stack[--sp];stack[sp-1]=min(stack[sp-1],b);break;}
    case OP_SQR:if(!sp)return false;stack[sp-1]=((int32_t)stack[sp-1] * stack[sp-1]) >> 8;break;
    case OP_DIST:{
      if(sp<6)return false;
      fixed z2=stack[--sp], y2=stack[--sp], x2=stack[--sp], z1=stack[--sp], y1=stack[--sp], x1=stack[--sp];
      int32_t dx=x2-x1, dy=y2-y1, dz=z2-z1;
      stack[sp++] = fx_sqrt((fixed)((dx*dx + dy*dy + dz*dz) >> 8));
      break;
    }
    default:return false;
  }}
  return sp ? (stack[sp-1] != 0) : false;
}
} // namespace V1FunctionConversion
