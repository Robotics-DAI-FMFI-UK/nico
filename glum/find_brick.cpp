#include <inttypes.h>


static int minr, maxr, mins, maxs;
uint8_t *buffer;
int sirka, vyska;

void zisti_rgb(int riadok, int stlpec, uint8_t *r, uint8_t *g, uint8_t *b)
{
  	      *r = buffer[riadok * sirka * 3 + stlpec * 3];
  	      *g = buffer[riadok * sirka * 3 + stlpec * 3 + 1];
  	      *b = buffer[riadok * sirka * 3 + stlpec * 3 + 2];
}

int je_lopta(int r, int g, int b)
{
  return (r > g * 2) && (r > b * 2);
}

int fill(int riadok, int stlpec)
{
  if (riadok < minr) minr = riadok;
  if (riadok > maxr) maxr = riadok;
  if (stlpec < mins) mins = stlpec;
  if (stlpec > maxs) maxs = stlpec;
  
  buffer[riadok * sirka * 3 + stlpec * 3] = 70;
  buffer[riadok * sirka * 3 + stlpec * 3 + 1] = 255;
  buffer[riadok * sirka * 3 + stlpec * 3 + 2] = 70;
  
  uint8_t r, g, b;
  
  zisti_rgb(riadok, stlpec + 1, &r, &g, &b);
  int kolko = 1;
  
  if (je_lopta(r, g, b))
    kolko += fill(riadok, stlpec + 1);

  zisti_rgb(riadok, stlpec - 1, &r, &g, &b);
  
  if (je_lopta(r, g, b))
    kolko += fill(riadok, stlpec - 1);

  zisti_rgb(riadok - 1, stlpec, &r, &g, &b);
    
  if (je_lopta(r, g, b))
    kolko += fill(riadok - 1, stlpec);

  zisti_rgb(riadok + 1, stlpec, &r, &g, &b);
  
  if (je_lopta(r, g, b))
    kolko += fill(riadok + 1, stlpec);

  return kolko;
}
 
void najdi_loptu(int *sirka_lopty, int *vyska_lopty, int *velkost_lopty, int *riadok, int *stlpec)
{
      const uint8_t *p = buffer;
      for (int i = 0; i < vyska; i++)
      {
         // lavy okraj
         buffer[i*sirka*3] = 0;
         buffer[i*sirka*3 + 1] = 0;
         buffer[i*sirka*3 + 2] = 0;
         
         // pravy okraj
         buffer[(i + 1)*sirka*3 - 3] = 0;
         buffer[(i + 1)*sirka*3 - 2] = 0;
         buffer[(i + 1)*sirka*3 - 1] = 0;
      }
      
      int index_zaciatku_dolneho_riadku = (vyska - 1) * sirka * 3;
      for (int i = 0; i < sirka; i++)
      {
         // horny okraj
         buffer[i*3] = 0;
         buffer[i*3 + 1] = 0;
         buffer[i*3 + 2] = 0;
         
         // dolny okraj
         buffer[index_zaciatku_dolneho_riadku + i * 3] = 0;
         buffer[index_zaciatku_dolneho_riadku + i * 3 + 1] = 0;
         buffer[index_zaciatku_dolneho_riadku + i * 3 + 2] = 0;
      }      

      int doteraz_najvacsi = 0;
      int doteraz_najv_sirka = 0;
      int doteraz_najv_vyska = 0;
      int doteraz_najv_riadok = 0;
      int doteraz_najv_stlpec = 0;
      
      for (int i = 0; i < vyska; i++)
        for (int j = 0; j < sirka; j++)
        {
  	      uint8_t r = *(p++);
  	      uint8_t g = *(p++);
  	      uint8_t b = *(p++);

  	      if (je_lopta(r, g, b))
  	      {
                  mins = sirka, minr = vyska, maxs = -1, maxs = -1;
                  int pocet = fill(i, j);
                  if (pocet > doteraz_najvacsi)
                  {
                      doteraz_najvacsi = pocet;
                      doteraz_najv_sirka = maxs - mins + 1;
                      doteraz_najv_vyska = maxr - minr + 1;
                      doteraz_najv_riadok = (maxr + minr) / 2;
                      doteraz_najv_stlpec = (maxs + mins) / 2;
                  }
  	      }
        }
    //  printf("velkost: %d, sirka: %d, vyska: %d\n", doteraz_najvacsi, 
    //           doteraz_najv_sirka, doteraz_najv_vyska);

      *sirka_lopty = doteraz_najv_sirka;
      *vyska_lopty = doteraz_najv_vyska;
      *velkost_lopty = doteraz_najvacsi;
      *riadok = doteraz_najv_riadok;
      *stlpec = doteraz_najv_stlpec;
}
