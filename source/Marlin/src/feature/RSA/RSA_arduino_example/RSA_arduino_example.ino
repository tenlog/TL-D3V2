#include <RSA.h>

char msg[PLAINTEXT_SIZE] = "1BqpadwA";

unsigned long publicKey[2] = {28841, 2591};		//14351, 11 // PQ 151 191
unsigned long privateKey[2] = {28841, 11};//14351, 1283 //PQ 113 127
void setup()
{
  Serial.begin(9600);
  Serial.println(String(PLAINTEXT_SIZE));
	Serial.println(msg);	

  for(int i = 0; i < PLAINTEXT_SIZE; i++){
    Serial.print(msg[i], DEC);
    Serial.print(",");
  }
  Serial.println("End");
  unsigned long cipher_msg[CIPHERTEXT_SIZE];	
	
	Serial.print(millis());
  Serial.println(" start enc");
	rsa.encrypt(msg, cipher_msg, publicKey);
	Serial.print(millis());
  Serial.println(" enc done");

  for(int i = 0; i < CIPHERTEXT_SIZE; i++){
    Serial.print(cipher_msg[i], DEC);
    Serial.print(",");
  }
	Serial.println("End");

	char plain[PLAINTEXT_SIZE];
	unsigned long cip[16] = {46,50,12,20,61,43,99,83,122,5,65,14,214,86,9,110};
	Serial.print(millis());
  Serial.println(" start dec");
  rsa.decrypt(plain, cip, privateKey);
	Serial.print(millis());
  Serial.println(" dec done");

  for(int i = 0; i < PLAINTEXT_SIZE; i++){
    Serial.print(plain[i], DEC);
    Serial.print(",");
  }
	Serial.println("End");

  Serial.println(plain);
}

void loop()
{
	delay(1000);
}