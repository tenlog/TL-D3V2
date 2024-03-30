
#include "../inc/MarlinConfig.h"

#if ENABLED(RSA_TEST)

#include "RSA.h"

RSA rsa;

RSA::RSA()
{

}

RSA::~RSA()
{

}

void RSA::encrypt(char *plainText, unsigned long *cipherText, unsigned long *publicKey)
{
   unsigned long m = 1;
   unsigned long n = publicKey[0];
   unsigned long e = publicKey[1];
   unsigned long ctr = 0;

   for(int i = 0; i < PLAINTEXT_SIZE; i++) {
       for(int j = 0; j < e; j++) {
           m = (m * plainText[i]) % n;
       }
       
       ctr = i * 2;//sizeof(unsigned long);

       cipherText[ctr]     = (unsigned char) (m & 0x00ff);
       cipherText[ctr + 1] = (unsigned char) ((m & 0xff00) >> 8);

       m = 1;
   }

#if DEBUG
   Serial.println("\n==========BEGIN CIPHERTEXT==========");
   for(int i = 0; i < CIPHERTEXT_SIZE; i++) {
       Serial.print((unsigned char)cipherText[i], DEC); Serial.print(",");
   }
   Serial.println("\n===========END CIPHERTEXT===========\n");
#endif
}

void RSA::decrypt(char *plainText, unsigned long *cipherText, unsigned long *privateKey)
{
   unsigned long M = 1;
   unsigned long n = privateKey[0];
   unsigned long d = privateKey[1];
   unsigned long temp = 0;
   unsigned long ctr = 0;

   //re-assemble char array to array of int
   for(int i = 0; i < PLAINTEXT_SIZE; i++) {
       ctr = i * 2;//sizeof(unsigned long);
       temp = (((unsigned char)cipherText[ctr + 1] << 8) | (unsigned char)cipherText[ctr]);
       
       for(int j = 0; j < d; j++) {
           M = (M * temp) % n;
       }

       plainText[i] = (unsigned char)(M & 0xFF); 
       M = 1;
   }

#if DEBUG
   Serial.println("\n==========BEGIN PLAINTEXT==========");
   for(int i = 0; i < PLAINTEXT_SIZE; i++)
       Serial.print(plainText[i]); 
   Serial.println("\n===========END PLAINTEXT===========\n");
#endif
}

bool RSA::compare(char *arr1, char *arr2, int len)
{
    int res = 0;
    
    res = memcmp(arr1, arr2, len);
    if(res == 0)
        return true;
    else 
        return false;
}

#endif
