#include <SPI.h>
#include <MFRC522.h> // 引用程式庫

#define RST_PIN A0 // 讀卡機的重置腳位
#define SS_PIN 10 // 晶片選擇腳位

MFRC522 mfrc522(SS_PIN, RST_PIN);  // 建立MFRC522物件

void setup() 
{
    Serial.begin(9600);

    SPI.begin();
    mfrc522.PCD_Init();   // 初始化MFRC522讀卡機模組
}

void loop() 
{
    // 確認是否有新卡片
    if (mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial()) 
    {
        byte *id = mfrc522.uid.uidByte; // 取得卡片的UID
        byte idSize = mfrc522.uid.size; // 取得UID的長度
 
        for (byte i = 0; i < idSize; i++) 
            Serial.print(id[i], DEC); // 以16進位顯示UID值
        Serial.println();
        
        mfrc522.PICC_HaltA();  // 讓卡片進入停止模式
        delay(1000);
    } 
}
