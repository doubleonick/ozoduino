float fmean(float arr[], int size) 
{
  if (size == 0) return 0.0;

  float sum = 0.0;
  for (int i = 0; i < size; i++) 
  {
    sum += arr[i];
  }
  return sum / size;
}

float fmode(float arr[], int size) 
{
  if (size == 0) return 0.0;

  float mode = arr[0];
  int maxCount = 1;

  for (int i = 0; i < size; i++) 
  {
    int count = 1;
    for (int j = i + 1; j < size; j++) 
    {
      if (arr[j] == arr[i]) 
      {
        count++;
      }
    }
    if (count > maxCount) 
    {
      maxCount = count;
      mode = arr[i];
    }
  }

  return mode;
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  float myArry[] = {9.0, 10.0, 11.0, 10.0};

  Serial.println(fmean(myArry, sizeof(myArry)));
  Serial.println(fmode(myArry, sizeof(myArry)));
}
