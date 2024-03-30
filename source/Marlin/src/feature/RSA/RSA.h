#ifndef __RSA__
#define __RSA__

#define DEBUG 0

#define SMS_SIZE            16
#define PLAINTEXT_SIZE      (SMS_SIZE / 2) //sizeof(unsigned long)
#define CIPHERTEXT_SIZE     (SMS_SIZE)

#include "Arduino.h"

class RSA {
    private:

    public:
        RSA();
        ~RSA();
        void encrypt(char *plainText, unsigned long *cipherText, unsigned long *publicKey);
        void decrypt(char *plainText, unsigned long *cipherText, unsigned long *privateKey);
        bool compare(char *arr1, char *arr2, int len);
};
extern RSA rsa;
#endif
