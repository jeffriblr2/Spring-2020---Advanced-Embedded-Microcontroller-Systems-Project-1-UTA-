void strCopy1(char *s1, char *s2)
{
while(*s1 != '\0')
{ *s2=*s1;
  s1++;
  s2++;
}
if(*s1=='\0')
    *s2='\0';

}

void strCopy2(const char *s1, char *s2)
{
while(*s1 != '\0')
{ *s2=*s1;
  s1++;
  s2++;
}
if(*s1=='\0')
    *s2='\0';

}

uint32_t power(int number, int exponent)
{uint32_t i,result=1;
 for(i=exponent;i>0;i--)
   result=result*number;
 return result;
}

uint8_t strLength(char *l)
{ uint8_t Length=0;
  while(*l!='\0')
 {l++;
 Length++;
 }
  return Length;
}

uint8_t strCompare(char *c1, char *c2)
{while((*c1!='\0')&&(*c2!='\0'))
 {if(*c1==*c2)
  {c1++;
   c2++;
  }
  else
    break;
  }
  if((*c1=='\0')&&(*c2=='\0'))
    return 0;
  else
    return 1;
 }

uint32_t ascii_to_integer(char *a)
{ uint32_t result=0,x;
uint32_t exp=strLength(a)-1;
uint32_t place=power(10,exp);
 while(*a!='\0')
 {if(*a==48)
  x=0;
  else if(*a==49)
   x=1;
  else if(*a==50)
   x=2;
  else if(*a==51)
   x=3;
  else if(*a==52)
   x=4;
  else if(*a==53)
   x=5;
  else if(*a==54)
   x=6;
  else if(*a==55)
   x=7;
  else if(*a==56)
   x=8;
  else if(*a==57)
   x=9;
 result =result+(x*place);
 place=place/10;
 if(place==0)
   place=1;
 a++;
 }
 return result;
}

void my_sprintf(char *ns, uint32_t x)
{
 uint32_t i,y,temp,e=0;
 temp=x;
 char str[11];
 if(x==0)
 {
  *ns=48;
  ns++;
 }
 else
 {
 while(x!=0)
 {
  x=x/10;
  e++;
 }
 //char str[e];
 x=temp;
 for(i=0;i<e;i++)
 {
  y=x%10;
  x=x/10;
  if(y==0)
  str[i]=48;
  else if(y==1)
   str[i]=49;
  else if(y==2)
   str[i]=50;
  else if(y==3)
   str[i]=51;
  else if(y==4)
   str[i]=52;
  else if(y==5)
   str[i]=53;
  else if(y==6)
   str[i]=54;
  else if(y==7)
   str[i]=55;
  else if(y==8)
   str[i]=56;
  else if(y==9)
   str[i]=57;
 }
 if(e>1)
 {
 for(i=(e-1);i>=1;i--)
 {
  *ns=str[i];
   ns++;
   if(i==1)
   { *ns=str[i-1];
      ns++;
   }
   }
 }
 else
 {
     *ns=str[0];
     ns++;
 }
 }

 // ns++;
 *ns='\0';
}

