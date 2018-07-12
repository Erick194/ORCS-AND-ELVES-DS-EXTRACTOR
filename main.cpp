#include <io.h>
#include <conio.h>
#include <png.h>
#include <algorithm>
#include <vector>       // std::vector

#include "funciones.h"
#include "Endian.h"
#include "zip.h"

using namespace std;

HZIP hz; DWORD writ;

enum
{
	RIFF_ID	= 'RIFF',
	WAVE_ID	= 'WAVE',
	FMT_ID	= 'fmt ',
	DATA_ID	= 'data',
};

typedef struct RIFFHeaderTag
{
	uint	RiffId;
	uint	Size;
	uint	WaveId;
} RIFFHeader;

typedef struct FMTHeaderTag
{
	uint	FmtId;
	uint	ChunkSize;
	word	Format;
	word	NumChannels;
	uint	SamplesPerSec;
	uint	AvgBytesPerSec;
	word	BlockAlign;
	word	BitsPerSample;
} FMTHeader;

typedef struct DATAHeaderTag
{
	uint	DataId;
	uint	ChunkSize;
} DATAHeader;

typedef struct
{
	char *name;
	int Offset;
	int size;
	bool Compress;
}RomFiles;

RomFiles Rom[10] = 
{
{"shaders.bin.lz", 0x580A00, 11958, true},
{"UI_Palette_Offset.bin", 0x9F000, 1248, false},
{"UI_Palettes.bin", 0x9F600, 103062, false},
{"UI_Shape_Offset.bin", 0xB8A00, 1332, false},
{"UI_Shapes.bin", 0xB9000, 1785029, false},
{"data.bin", 0x584800, 4069960, false},
{"headers.bin", 0x966400, 7012, false},
//{"models.bin.lz", 0x9C1800, 95424, true},
{"palettes.bin.lz", 0x9D8E00, 97681, true},
{"texels.bin", 0x9F0C00, 1136775, false}
};

typedef struct
{
	int Mode;
	int ClipW;
	int ClipH;
	int Width;
	int Height;
	int Count;
	int Size;
	int Type;
	int Offset;
	int OutSize;
}Graphic;

typedef struct
{
	int Size;
	int Count;
	int Offset;
	int OutSize;
}Palette;

typedef struct
{
	int Offset;
	int OutSize;
}UIOffset;

typedef struct
{
	int Offset;
	int OutSize;
	int HMZ;
}SFXINFO;

unsigned short Images[357][2] =
{
// texidx, palidx
{0,0},
{1,1},
{2,2},
{3,3},
{4,4},
{5,5},
{6,6},
{7,7},
{8,8},
{9,9},
{10,10},
{11,11},
{12,12},
{13,13},
{14,14},
{15,15},
{16,16},
{17,17},
{18,18},
{19,19},
{20,20},
{21,21},
{22,22},
{23,19},
{24,23},
{25,24},
{26,25},
{27,26},
{28,27},
{29,28},
{30,29},
{31,30},
{32,31},
{33,32},
{34,33},
{35,34},
{36,35},
{37,36},
{38,37},
{39,38},
{40,37},
{41,38},
{42,39},
{43,40},
{44,41},
{45,42},
{46,43},
{47,44},
{48,43},
{49,42},
{50,42},
{51,45},
{52,46},
{53,47},
{54,48},
{55,49},
{56,50},
{57,51},
{58,51},
{59,51},
{60,51},
{61,51},
{62,51},
{63,52},
{64,53},
{65,52},
{66,54},
{67,55},
{68,56},
{69,57},
{70,58},
{71,59},
{72,60},
{73,61},
{74,62},
{75,63},
{76,64},
{77,65},
{78,66},
{79,67},
{80,68},
{81,69},
{82,70},
{83,71},
{84,72},
{85,73},
{86,74},
{87,75},
{88,76},
{89,77},
{90,78},
{91,79},
{92,79},
{93,79},
{94,79},
{95,80},
{95,83},
{95,85},
{95,87},
{96,81},
{96,84},
{96,86},
{96,88},
{97,82},
{98,80},
{98,83},
{98,85},
{98,87},
{99,89},
{100,89},
{101,90},
{102,91},
{103,92},
{104,93},
{105,93},
{106,94},
{106,95},
{106,96},
{107,97},
{108,98},
{109,99},
{110,100},
{111,101},
{112,102},
{113,103},
{113,108},
{113,110},
{113,111},
{113,113},
{113,115},
{113,116},
{113,117},
{114,104},
{114,106},
{114,109},
{115,105},
{115,107},
{115,112},
{115,114},
{116,118},
{116,137},
{117,119},
{118,120},
{119,121},
{120,122},
{121,123},
{122,124},
{123,125},
{124,126},
{125,127},
{126,128},
{127,129},
{128,130},
{129,131},
{130,132},
{131,133},
{132,134},
{133,135},
{134,136},
{135,138},
{136,139},
{137,140},
{138,141},
{139,142},
{140,143},
{141,144},
{142,145},
{143,146},
{144,144},
{145,145},
{146,146},
{147,146},
{148,143},
{149,146},
{150,143},
{151,143},
{152,146},
{153,144},
{154,146},
{155,143},
{156,146},
{157,146},
{158,147},
{159,148},
{160,149},
{161,150},
{162,147},
{163,151},
{164,152},
{165,153},
{166,154},
{167,153},
{168,155},
{169,156},
{170,157},
{171,158},
{172,159},
{173,160},
{174,161},
{175,162},
{176,163},
{177,164},
{178,165},
{179,166},
{180,167},
{181,168},
{182,169},
{183,170},
{184,171},
{185,172},
{186,173},
{187,174},
{188,175},
{189,176},
{190,177},
{191,178},
{192,179},
{193,180},
{194,181},
{195,182},
{196,181},
{197,183},
{198,184},
{199,185},
{200,186},
{201,187},
{202,188},
{203,189},
{204,190},
{205,191},
{206,192},
{207,193},
{208,194},
{209,195},
{210,196},
{211,197},
{212,198},
{213,199},
{214,200},
{215,201},
{216,202},
{217,203},
{218,204},
{219,205},
{220,206},
{221,207},
{222,208},
{223,209},
{224,210},
{225,211},
{226,212},
{227,213},
{228,214},
{229,215},
{230,216},
{231,217},
{232,218},
{233,219},
{234,220},
{235,221},
{236,222},
{237,223},
{238,224},
{239,225},
{240,226},
{241,227},
{242,228},
{243,229},
{244,229},
{245,230},
{246,231},
{247,232},
{248,233},
{249,234},
{250,235},
{251,236},
{252,237},
{253,238},
{254,239},
{255,240},
{256,241},
{257,242},
{258,243},
{259,244},
{260,245},
{261,246},
{262,247},
{263,248},
{264,249},
{265,250},
{266,251},
{267,252},
{268,253},
{269,254},
{270,255},
{271,256},
{272,257},
{273,258},
{274,259},
{275,260},
{276,261},
{277,262},
{278,263},
{279,264},
{280,265},
{281,266},
{282,267},
{283,268},
{284,269},
{285,270},
{286,271},
{287,272},
{288,273},
{289,272},
{290,274},
{291,275},
{292,276},
{293,277},
{294,278},
{295,277},
{296,278},
{297,277},
{298,275},
{299,277},
{300,279},
{301,280},
{302,281},
{303,282},
{304,283},
{305,284},
{306,285},
{307,286},
{308,287},
{309,288},
{310,289},
{311,290},
{312,291},
{313,292},
{314,293},
{315,294},
{316,295},
{317,296},
{318,297},
{319,298},
{320,299},
{321,300},
{322,301},
{323,302},
{324,303},
{325,304},
{326,305},
{327,306},
{328,307},
{329,308},
{330,309},
{331,310},
{332,311},
};

Graphic *Graphics;
Palette *Palettes;
Palette *UPalettes;
UIOffset *UShapeOffset;

typedef byte*  cache;
char name[32] = { 0 };

static const int N = 4096, F = 18;
static const unsigned char THRESHOLD = 2;
static const int NIL = N;
static const int LZ77_TAG = 0x10;

static int MAX_OUTSIZE = 0x200000;
static long long decomp_size, curr_size;

int DecompressLZ77(byte *input, byte *output)
{
		decomp_size = 0, curr_size = 0;
		int flags, i, j, disp, n;
		
		bool flag;
		unsigned char b;
		long long cdest;
		int bits=0;
		if (/*getc(fstr)*/*input++ != LZ77_TAG)
		{
			//fseek(fstr,4,0);
			//if (getc(fstr) != LZ77_TAG)
			{
				throw printf("El archivo %s no es un archivo LZ77 valido\n"/*, filein*/);
			}
		}
		for (i = 0; i < 3; i++)
		{
			decomp_size += /*getc(fstr)*/(*input++) << (i * 8);
		}
		bits+=3;
		if (decomp_size > MAX_OUTSIZE)
		{
			throw printf("%s sera mas largo que 0x1%x 0x2%x y no puede ser descomprimido."/*,filein*/, MAX_OUTSIZE, decomp_size);
		}
		else if (decomp_size == 0)
		{
			for (i = 0; i < 4; i++)
			{
			}
		}
		if (decomp_size > MAX_OUTSIZE << 8)
		{
			throw printf("%s sera mas largo que 0x1%x 0x2%x y no puede ser descomprimido."/*,filein*/, MAX_OUTSIZE, decomp_size);
		}
		/*if (showAlways)
		{
			printf("Descomprimiendo ");
		}*/
		unsigned char outdata[decomp_size];
		while (curr_size < decomp_size)
		{
			try
			{
				flags = (*input++);//getc(fstr);
				bits++;
			}
			catch (char e1)
			{
				break;
			}
			for (i = 0; i < 8; i++)
			{
				flag = (flags & (0x80 >> i)) > 0;
				if (flag)
				{
					disp = 0;
					try
					{
						b = (*input++);//getc(fstr);
						bits++;
					}
					catch (char e2)
					{
					throw printf("Datos incompletos");
					}
					n = b >> 4;
					disp = (b & 0x0F) << 8;
					try
					{
						disp |= (*input++);//getc(fstr);
						bits++;
					}
					catch (char  e3)
					{
					throw printf("Datos incompletos");
					}
					n += 3;
					cdest = curr_size;
					if (disp > curr_size)
					{
						throw printf("Cannot go back more than already written");
					}
					for (j = 0; j < n; j++)
					{
						outdata[curr_size++] = outdata[cdest - disp - 1 + j];
					}
					if (curr_size > decomp_size)
					{
						break;
					}
				}
				else
				{
					try
					{
						b = (*input++);//getc(fstr);
						bits++;
					}
					catch (char  e4)
					{
						break;
					}
					try
					{
						outdata[curr_size++] = b;
					}
					catch (int e5)
					{
						if (b == 0)
						{
							break;
						}
					}
					if (curr_size > decomp_size)
					{
						break;
					}
				}
			}
		}
		try
		{
			while (/*getc(fstr)*/(*input++) == 0)
			{
            //bits++;
			}
		}
		catch (char  e6)
		{
		}
		
		for(int i = 0; i < decomp_size; i++)
            output[i] = outdata[i];

		//printf("LZ77 Descomprimido %d\n",bits);
}

static cache writeData;
static unsigned int current = 0;

//**************************************************************
//**************************************************************
//  Png_WriteData
//
//  Work with data writing through memory
//**************************************************************
//**************************************************************

static void Png_WriteData(png_structp png_ptr, cache data, size_t length) {
    writeData = (byte*)realloc(writeData, current + length);
    memcpy(writeData + current, data, length);
    current += length;
}

//**************************************************************
//**************************************************************
//  Png_Create
//
//  Create a PNG image through memory
//**************************************************************
//**************************************************************

cache Png_Create(bool texture, cache data, int* size, int width, int height, int paloffset, int palsize, int mode, int offsetx = 0, int offsety = 0)
{
    int i, j;
    cache image;
    cache out;
    cache* row_pointers;
    png_structp png_ptr;
    png_infop info_ptr;
    png_colorp palette;
    png_colorp newpalette;
    
    int bit_depth = 8;
    int pal_depth = 256;

    if(texture == false)
    {
        if(mode == 4)
        {
         bit_depth = 4;
         pal_depth = 16;
        }
    }
    else
    {
        if(mode == 3)
        {
         bit_depth = 4;
         pal_depth = 16;
        }
    }
    
    // setup png pointer
    png_ptr = png_create_write_struct(PNG_LIBPNG_VER_STRING, 0, 0, 0);
    if(png_ptr == NULL) {
        error("Png_Create: Failed getting png_ptr");
        return NULL;
    }

    // setup info pointer
    info_ptr = png_create_info_struct(png_ptr);
    if(info_ptr == NULL) {
        png_destroy_write_struct(&png_ptr, NULL);
        error("Png_Create: Failed getting info_ptr");
        return NULL;
    }

    // what does this do again?
    if(setjmp(png_jmpbuf(png_ptr))) {
        png_destroy_write_struct(&png_ptr, &info_ptr);
        error("Png_Create: Failed on setjmp");
        return NULL;
    }

    // setup custom data writing procedure
    png_set_write_fn(png_ptr, NULL, Png_WriteData, NULL);

    // setup image
    png_set_IHDR(
        png_ptr,
        info_ptr,
        width,
        height,
        bit_depth,
        PNG_COLOR_TYPE_PALETTE,
        PNG_INTERLACE_NONE,
        PNG_COMPRESSION_TYPE_DEFAULT,
        PNG_FILTER_TYPE_DEFAULT);

    // setup palette
    FILE *fpal = fopen ("RomData/palettes.bin.lz","rb");
    if(!fpal)
       error("No puede abrir palettes.bin.lz");
    
    if(texture == false)
    {
       fclose(fpal);
       
       fpal = fopen ("RomData/UI_Palettes.bin","rb");
       if(!fpal)
          error("No puede abrir UI_Palettes.bin");
    }
    
    //printf("MODE %d\n",mode);

    fseek(fpal,paloffset,SEEK_SET);
    palette = (png_colorp) malloc((pal_depth*3)*png_sizeof(png_color));
    memset (palette,0x00,(pal_depth*3)*png_sizeof(png_color));
    
    
     
    if(texture == false)
    {
       palsize = ReadByte(fpal);
       if(palsize == 0)
       {
        palsize = 256;
       }
    }
     
    int last = 0;
    int cnt = 0;
    int gettrans = -1;
    for(int x = 0;x < palsize;x++)
    {
       int Palbit = ReadWord(fpal);
       int R = ((Palbit >> 0  & 0x1F) << 3);
       int G = ((Palbit >> 5  & 0x1F) << 3);
       int B = ((Palbit >> 10 & 0x1F) << 3);

       palette[x].red = R;
       palette[x].green = G;
       palette[x].blue = B;

       if(mode == 1)
       {
          last = x;
          gettrans = 255;
       }
       else if(mode == 6)
       {
          //if( x % 2 == 0)
          if(x == 8)
          if(palette[x].red==248 && palette[x].green==192 && palette[x].blue==56)
          {
            last = x-1; 
            gettrans = 255;   
            //gettrans = 0;            
            break;
          }
       }
       else
       {
          if(texture == false)
          {
              if(palette[x].red==248 && palette[x].green==0 && palette[x].blue==248)
              gettrans = x;
          }
          else
          {
              if(palette[0].red==248 && palette[0].green==248 && palette[0].blue==248)
              gettrans = 0;
              if(palette[0].red==248 && palette[0].green==0 && palette[0].blue==248)
              gettrans = 0;
              if(palette[0].red==240 && palette[0].green==0 && palette[0].blue==248)
              gettrans = 0;
              if(palette[0].red==232 && palette[0].green==0 && palette[0].blue==248)
              gettrans = 0;
              if(palette[0].red==248 && palette[0].green==0 && palette[0].blue==0)
              gettrans = 0;
              if(palette[0].red==0 && palette[0].green==248 && palette[0].blue==248)
              gettrans = 0;
          }
       }
       
    }
    fclose(fpal);

    if(mode == 6 || mode == 1)
    {
        newpalette = (png_colorp) malloc((pal_depth*3)*png_sizeof(png_color));
        memset (newpalette,0x00,(pal_depth*3)*png_sizeof(png_color));
        
        for(int x = 0;x < pal_depth;x++)
        {
           if(cnt > last)cnt=0;
           
           int R = palette[cnt].red * x / 255;
           int G = palette[cnt].green * x / 255;
           int B = palette[cnt].blue * x / 255;

           //if(mode == 1)
           if(mode == 6 || mode == 1)
           {
           R = palette[cnt].red;
           G = palette[cnt].green;
           B = palette[cnt].blue;
           }
           
           newpalette[x].red = R;
           newpalette[x].green = G;
           newpalette[x].blue = B;
           cnt++;
           
           //if(newpalette[x].red==0 && newpalette[x].green==255 && newpalette[x].blue==255)
              //gettrans = x;
        }
        //printf("MODE %d\n",mode);
        png_set_PLTE(png_ptr, info_ptr,newpalette,pal_depth);
    }
    else
    {
     png_set_PLTE(png_ptr, info_ptr,palette,pal_depth);
    }
    
    //if(mode == 1)
    if(mode == 6 || mode == 1)
    {
        png_byte trans[gettrans+1]; 
        for(int tr =0;tr < gettrans+1; tr++)
        {
          /*if(tr==gettrans){trans[tr]=tr;}
          else {trans[tr]=255;}*/
          if(bit_depth == 4)
            trans[tr] = tr*16;
          else
            trans[tr] = tr;
            
          if(tr >= ((gettrans+1)-palsize))
            trans[tr] = 255;
        }
        png_set_tRNS(png_ptr, info_ptr,trans,gettrans+1,NULL);
    }
    else
    {
        if(gettrans != -1)
        {
            png_byte trans[gettrans+1]; 
            for(int tr =0;tr < gettrans+1; tr++)
            {
              if(tr==gettrans){trans[tr]=0;}
              else {trans[tr]=255;}
            }
            png_set_tRNS(png_ptr, info_ptr,trans,gettrans+1,NULL);
        }
    }

    // add png info to data
    png_write_info(png_ptr, info_ptr);
    
    if(offsetx !=0 || offsety !=0)
    {
       int offs[2];
    
       offs[0] = Swap32(offsetx);
       offs[1] = Swap32(offsety);
    
       png_write_chunk(png_ptr, (png_byte*)"grAb", (byte*)offs, 8);
    }

    // setup packing if needed
    png_set_packing(png_ptr);
    png_set_packswap(png_ptr);

    // copy data over
    byte inputdata;
    image = data;
    row_pointers = (cache*)malloc(sizeof(byte*) * height);
    for(i = 0; i < height; i++)
    {
        row_pointers[i] = (cache)malloc(width);
        for(j = 0; j < width; j++)
        {
            inputdata = *image;
            //if(inputdata == 0x7f){inputdata = (byte)gettrans;}
            row_pointers[i][j] = inputdata;
            image++;
        }
    }

    // cleanup
    png_write_image(png_ptr, row_pointers);
    png_write_end(png_ptr, info_ptr);
    free((void**)&palette);
    free((void**)&newpalette);
    free((void**)row_pointers);
    palette = NULL;
    row_pointers = NULL;
    png_destroy_write_struct(&png_ptr, &info_ptr);

    // allocate output
    out = (cache)malloc(current);
    memcpy(out, writeData, current);
    *size = current;

    free(writeData);
    writeData = NULL;
    current = 0;

    return out;
}

void ExtraerRomFiles()
{
   int i, j;
   cache input;
   cache output;
   
   FILE *rom = fopen("Orcs and Elves.nds","rb");
   if(!rom)
     error("No puede abrir Orcs and Elves.nds");
   
   mkdir("RomData");
   
   for(i = 0; i < 9/*10*/; i++)
   {
       PrintfPorcentaje(i,/*10*/9-1,true, 8,"Extrayendo Archivos......");
       fseek(rom, Rom[i].Offset, SEEK_SET);
       
       sprintf(name,"RomData/%s",Rom[i].name);
       
       FILE *data = fopen(name,"wb");
       
       if(Rom[i].Compress)
       {
           input = (byte*)malloc(Rom[i].size);
           output = (byte*)malloc(MAX_OUTSIZE);
       }
       
       for(j = 0; j < Rom[i].size; j++)
       {
           if(Rom[i].Compress)
               input[j] = ReadByte(rom);
           else
               fputc(ReadByte(rom),data);
       }

       if(Rom[i].Compress)
       {
           DecompressLZ77(input,output);
           
           //printf("decomp_size %d\n",decomp_size);
           for(j = 0; j < decomp_size; j++)
               fputc(output[j],data);
               
           free(input);
           free(output);
       }
       
       fclose(data);
       //printf("%s, %d, %d, %d\n",Rom[i].name,Rom[i].Offset,Rom[i].size,Rom[i].Compress);
   }
   
   fclose(rom);
}

void Load_UI_Palette_Offset()
{
    FILE *in;
    in = fopen("RomData/UI_Palette_Offset.bin","rb");
    if(!in)
     error("No puede abrir UI_Palette_Offset.bin");
     
    int long palsize = GetFileSize(in)/4;
    
    int LastSize = 0;
    int Count = 0;
    int Offset = 0;
    int Size = 0;
    
    if (palsize)
    {
        UPalettes = (Palette *)malloc (palsize * sizeof(Palette));
        do
        {
          Size = ReadUint(in);
          UPalettes[Count].Size = (Size - LastSize) >> 1;
          UPalettes[Count].Count = Size;
          LastSize = Size;
          UPalettes[Count].Offset = Offset;
          UPalettes[Count].OutSize = (UPalettes[Count].Count - Offset);
          Offset = UPalettes[Count].Count;
          ++Count;
        }
        while(Count < palsize);
    }
    fclose(in);
}

void Load_UI_Shape_Offset()
{
    FILE *in;
    in = fopen("RomData/UI_Shape_Offset.bin","rb");
    if(!in)
     error("No puede abrir UI_Shape_Offset.bin");
     
    int long shapesize = GetFileSize(in)/4;

    int LastSize = 0;
    int Count = 0;
    int Offset = 0;
    int Size = 0;
    
    //UIOffset *UShapeOffset;
    if (shapesize)
    {
        UShapeOffset = (UIOffset *)malloc (shapesize * sizeof(UIOffset));
        do
        {
          Size = ReadUint(in);
          UShapeOffset[Count].Offset = Offset;
          UShapeOffset[Count].OutSize = (Size - Offset);
          Offset = Size;
          ++Count;
        }
        while(Count < shapesize);
    }
    fclose(in);
}

static int id = 0;
static int palls; 
int gettarns(int palidx)
{
    // setup palette
    FILE *fpal = fopen ("RomData/UI_Palettes.bin","rb");
    if(!fpal)
    {
       error("No puede abrir UI_Palettes.bin");
    }

    fseek(fpal,UPalettes[palidx].Offset,SEEK_SET);
    
    int Size = ReadByte(fpal);
    if(Size == 0)Size = 256;
    palls = Size;

    int gettrans = 0;
    for(int x = 0;x < Size;x++)
    {
       int Palbit = ReadWord(fpal);
       int R = ((Palbit >> 0  & 0x1F) << 3);
       int G = ((Palbit >> 5  & 0x1F) << 3);
       int B = ((Palbit >> 10 & 0x1F) << 3);

       if(R==248 && G==0 && B==248)
       {
         gettrans = x;
       }
    }
    
    fclose(fpal);
    return gettrans;
}

static int tx, pl;
void ExtraerImages(int texidx, int palidx)
{
    int i, j, k;
    FILE *in = fopen("RomData/UI_Shapes.bin","rb");
    if(!in)
     error("No puede abrir UI_Shapes.bin");

    fseek(in, UShapeOffset[texidx].Offset, SEEK_SET);
    
    int Mode = ReadByte(in);
    int Width = ReadWord(in);
    int Height = ReadWord(in);
    int ShapeCnt = ReadWord(in);

    cache input;
    input = (byte*)malloc(Width*Height);
    int trans = gettarns(palidx);
    memset (input,trans,(Width*Height));
    
    int pixblank = 0xff;
    if(trans == 255)
          pixblank = 0x00;
          
    if((palls > 16) && (Mode == 4))
    {
     fclose(in);
     free(input);
     id++;
     return;
    }
    
    int line = 0 * Width;
    int position = line * Width;
    i = 0;
    do
    {
          int LineCnt = ReadByte(in);
          i++;
          
          position = line * Width;
          if(LineCnt == 0) line++;
          else
          {
              int Cnt = ReadWord(in);
              i+=2;
              for(j = 0; j < LineCnt; j++)
              {
                    int pos = ReadByte(in);
                    i++;
                    int pixcnt = ReadByte(in);
                    i++;
                    if(pixcnt == 0)pixcnt = 256;
                    for(k = 0; k < pixcnt; k++)
                    {
                          input[(position+pos)+k] = pixblank;
                    }
              }
              line++;
          }
          //getch();
    }
    while(i < ShapeCnt);
    
    int restsize = UShapeOffset[texidx].OutSize - (ShapeCnt+7);
    
    cache input4;
    input4 = (byte*)malloc(restsize*2);
    if(Mode == 4)
    {
        for(i = 0; i < (restsize*2); i+=2)
        {
            byte pixel = ReadByte(in);
            input4[i] = (byte)(pixel & 0xF);
            input4[i+1] = (byte)(pixel >> 4 & 0xF);
        }
    }
    else
    free(input4);
    
    int incnt = 0;
    for(i = 0; i < (Width*Height); i++)
    {
        if(input[i] == pixblank)
        {
            if(Mode == 4)
                input[i] = input4[incnt];
            else
                input[i] = ReadByte(in);
            incnt++;
        }
    }
    
    fclose(in);

    cache pngout;
    int pngsize = 0;
    pngout = Png_Create(false, input, &pngsize, Width, Height, UPalettes[palidx].Offset, 0 ,Mode);

    sprintf(name,"IMAGES/IMAG%03d",id);
    ZipAdd(hz, name, pngout, pngsize);
    id++;
    
    free(pngout);
    free(input);
}

byte *ReverseBytes( byte *start, int size , int width)
{
    byte *istart = start;
    
    std::vector<byte> data;
    for (int i=0; i<size; ++i)
    {
     data.push_back(istart[i]);
    }
    
    std::reverse(data.begin(),data.end());

    // Remapping loop
	int i, j, k;
	for (i = 0; i < size; ++i)
	{
		//if (vertical)
		//	j = (((height - 1) - (i/width)) * width) + (i%width);
		//else // horizontal
			j = ((i/width) * width) + ((width - 1) - (i%width));
		for (k = 0; k < 1; ++k)
		{
			istart[(j*1)+k] = data[(i*1)+k];
		}
	}
    
    return istart;
}

void ExtraerTexels(int texidx, int palidx)
{
    FILE *in = fopen("RomData/texels.bin","rb");
    
    if(!in)
     error("No puede abrir texels.bin");

    fseek(in, Graphics[texidx].Offset, SEEK_SET);
    
    cache input;
    cache output;
    cache output2;
    
    input = (byte*)malloc(Graphics[texidx].OutSize);
    output = (byte*)malloc(Graphics[texidx].Size);
    
    for(int i = 0; i < Graphics[texidx].OutSize; i++)
         input[i] = (byte)ReadByte(in);
         
    fclose(in);

    DecompressLZ77(input,output);

    int div = 1;
    if(Graphics[texidx].Mode == 3)
     div = 2;
     
    int W = Graphics[texidx].Width/div;
    
    output = ReverseBytes( output, Graphics[texidx].Size, W);
 
    int width = Graphics[texidx].Width;
    int height = Graphics[texidx].Height;
    
    output2 = (byte*)malloc(width*height);
    
    
    int cnt = 0;
    if(Graphics[texidx].Mode == 3)
    {
        for(int i = 0; i < (width*height); i+=2)
        {
            byte pixel = output[cnt]; cnt++;
            output2[i] = (byte)(pixel & 0xF);
            output2[i+1] = (byte)(pixel >> 4 & 0xF);
        }
    }
    else
    {
        for(int i = 0; i < (width*height); i++)
        {
            output2[i] = output[i];
        }
    }

    int offsetx = 0;
    int offsety = 0;

    if(Graphics[texidx].Type == 64)
    {
        offsetx = width/2;
        offsety = height;
    }
    
    cache pngout;
    int pngsize = 0;
    pngout = Png_Create(true, output2, &pngsize, width, height, Palettes[palidx].Offset,Palettes[palidx].Size,Graphics[texidx].Mode, offsetx, offsety);

    sprintf(name,"TEXTURES/TEXL%03d",id);
    ZipAdd(hz, name, pngout, pngsize);
    id++;
    
    free(input);
    free(output);
    free(output2);
    free(pngout);
}

SFXINFO *SFX;
static int sfxcnt = 0;

void LoadHeaders()
{
     FILE *in;
    in = fopen("RomData/headers.bin","rb");
    if(!in)
     error("No puede abrir headers.bin");
    
    sfxcnt = ReadUint(in);
    
    SFX = (SFXINFO *)malloc (sfxcnt * sizeof(SFXINFO));
    
    int nextoff = 0;
    int offset = 0;
    for(int i = 0; i < sfxcnt; i++)
    {
    
    int vv1 = ReadWord(in);
    int vv2 = ReadWord(in);
    int vv3 = ReadWord(in);
    int numid = ReadWord(in);
    int vv4 = ReadWord(in);
    int vv5 = ReadWord(in);
    int vv6 = ReadWord(in);
    int vv7 = ReadWord(in);
    int hmz = ReadUint(in);
    nextoff = ReadUint(in);
    
    SFX[i].Offset = offset;
    SFX[i].OutSize = (nextoff - offset);
    SFX[i].HMZ = hmz;
    offset = nextoff;
    
    /*printf("\nvv1 %d\n",vv1);
    printf("vv2 %d\n",vv2);
    printf("vv3 %d\n",vv3);
    printf("numid %d\n",numid);
    printf("vv4 %d\n",vv4);
    printf("vv5 %d\n",vv5);
    printf("vv6 %d\n",vv6);
    printf("vv7 %d\n",vv7);
    printf("hmz %d\n",SFX[i].HMZ);
    printf("Soffset %d\n",SFX[i].Offset);
    printf("size %d\n",SFX[i].OutSize);
    getch();*/

    }
    fclose(in); 
}

static int IndexTable[8] =
{
	-1, -1, -1, -1, 2, 4, 6, 8,
};

static int AdpcmTable[89] =
{
    0x0007, 0x0008, 0x0009, 0x000A, 0x000B, 0x000C, 0x000D, 0x000E, 0x0010, 0x0011, 0x0013, 0x0015,
    0x0017, 0x0019, 0x001C, 0x001F, 0x0022, 0x0025, 0x0029, 0x002D, 0x0032, 0x0037, 0x003C, 0x0042,
    0x0049, 0x0050, 0x0058, 0x0061, 0x006B, 0x0076, 0x0082, 0x008F, 0x009D, 0x00AD, 0x00BE, 0x00D1,
    0x00E6, 0x00FD, 0x0117, 0x0133, 0x0151, 0x0173, 0x0198, 0x01C1, 0x01EE, 0x0220, 0x0256, 0x0292,
    0x02D4, 0x031C, 0x036C, 0x03C3, 0x0424, 0x048E, 0x0502, 0x0583, 0x0610, 0x06AB, 0x0756, 0x0812,
    0x08E0, 0x09C3, 0x0ABD, 0x0BD0, 0x0CFF, 0x0E4C, 0x0FBA, 0x114C, 0x1307, 0x14EE, 0x1706, 0x1954,
    0x1BDC, 0x1EA5, 0x21B6, 0x2515, 0x28CA, 0x2CDF, 0x315B, 0x364B, 0x3BB9, 0x41B2, 0x4844, 0x4F7E,
    0x5771, 0x602F, 0x69CE, 0x7462, 0x7FFF
};
 
void DecodeIMA(byte* input, int numSamples, sword* output)
{
    byte *inp;		
    sword *outp;		
    int Index = 0;
    int Pcm16bit = 0;
    
    int Data = 0;
    int inputbuffer = 0;		
    int bufferstep = 0;	
    
    outp = output;
    inp = input;

    for ( ; numSamples > 0 ; numSamples-- )
    {
    	if ( bufferstep ){ Data = inputbuffer & 0xf; }
        else{ inputbuffer = *inp++; Data = (inputbuffer >> 4)& 0xf; }
    	bufferstep = !bufferstep;

        int Diff = ((Data &7)*2+1)*AdpcmTable[Index]/8;
        if ((Data & 8) == 0) Pcm16bit += Diff;
        if ((Data & 8) == 8) Pcm16bit -= Diff;
        if (Pcm16bit > 32767) Pcm16bit = 32767;
        else if (Pcm16bit < -32768) Pcm16bit = -32768;
        
        Index += IndexTable[Data & 7];
        if (Index < 0) Index = 0;
        if (Index > 88) Index = 88;

        Diff = AdpcmTable[Index]/8;
        if ((Data & 1) == 1) Diff += AdpcmTable[Index]/4;
        if ((Data & 2) == 2) Diff += AdpcmTable[Index]/2;
        if ((Data & 4) == 4) Diff += AdpcmTable[Index]/1;

        *outp++ = (short)Pcm16bit;
    }
}

void ExtraerSFX()
{
    FILE *in = fopen("RomData/data.bin","rb");
    
    if(!in)
     error("No puede abrir data.bin");
    
    cache input;

    for(int i = 0; i < sfxcnt; i++)
    {
        PrintfPorcentaje(i,sfxcnt-1,true, 11,"Extrayendo Sonidos SFX......");
        
        fseek(in, SFX[i].Offset, SEEK_SET);
        
        input = (byte*)malloc(SFX[i].OutSize);

        for(int j = 0; j < SFX[i].OutSize; j++)
             input[j] = (byte)ReadByte(in);

        int numSamples = SFX[i].OutSize * 2;
    
    	int outBufferSize = sizeof(RIFFHeader) + sizeof(FMTHeader) + sizeof(DATAHeader) + numSamples * 2;
    
    	byte* outFileBuf = (byte*) malloc(outBufferSize);
    
    	RIFFHeader* riffHeader = (RIFFHeader*) outFileBuf;
    
    	endianWriteU32Big(&riffHeader->RiffId, RIFF_ID);
    	endianWriteU32Little(&riffHeader->Size, outBufferSize - sizeof(RIFFHeader) + 4);
    	endianWriteU32Big(&riffHeader->WaveId, WAVE_ID);
    
    	FMTHeader* fmtHeader = (FMTHeader*) (riffHeader + 1);
    
    	endianWriteU32Big(&fmtHeader->FmtId, FMT_ID);
    	endianWriteU32Little(&fmtHeader->ChunkSize, sizeof(FMTHeader) - 8);
    	endianWriteU16Little(&fmtHeader->Format, 1);
    	endianWriteU16Little(&fmtHeader->NumChannels, 1);
    	endianWriteU32Little(&fmtHeader->SamplesPerSec, SFX[i].HMZ);
    	endianWriteU32Little(&fmtHeader->AvgBytesPerSec, SFX[i].HMZ*2);
    	endianWriteU16Little(&fmtHeader->BlockAlign, 2);
    	endianWriteU16Little(&fmtHeader->BitsPerSample, 16);
    
    	DATAHeader* dataHeader = (DATAHeader*) (fmtHeader + 1);
    
    	endianWriteU32Big(&dataHeader->DataId, DATA_ID);
    	endianWriteU32Little(&dataHeader->ChunkSize, numSamples * 2);
    	
    	DecodeIMA(input, numSamples,(sword*)(dataHeader + 1));

        sprintf(name,"SOUNDS/SFX%03d", i);
        ZipAdd(hz, name, outFileBuf, outBufferSize);
        
        free(input);
        free(outFileBuf);
    }
    
    fclose(in);
}

void ShowInfo()
{
    setcolor(0x07);printf("     ############");
    setcolor(0x0A);printf("(ERICK194)");
    setcolor(0x07);printf("#############\n"); 
    printf("     #    ORCS & ELVES DS EXTRACTOR    #\n");
    printf("     # CREADO POR ERICK VASQUEZ GARCIA #\n");
    printf("     #    ES PARA ORCS AND ELVES.NDS   #\n");
    printf("     #          MODO DE USO:           #\n");
    printf("     #  SOLO NECESITAS EL ARCHIVO NDS  #\n");
    printf("     ###################################\n");
    printf("\n");
}

int main(int argc, char *argv[])
{
    FILE *in;
    
    ShowInfo();

    in = fopen("RomData/shaders.bin.lz","rb");
    if(!in)
    {
     ExtraerRomFiles();
    }
    fclose(in);

    Load_UI_Palette_Offset();
    Load_UI_Shape_Offset();
    
    hz = CreateZip("OrcsAndElvesDS.pk3",0);
    
    for(int i = 0; i < 357;i++)
    {
        PrintfPorcentaje(i,357-1,true, 9,"Extrayendo Imagenes......");
        ExtraerImages(Images[i][0], Images[i][1]);
    }

    id = 0;
    
    in = fopen("RomData/shaders.bin.lz","rb");
    if(!in)
     error("No puede abrir shaders.bin.lz");
     
    int v44 = 0;
    int count1 = ReadUint(in);
    int count2 = ReadUint(in);
    int count3 = ReadUint(in);
    int count4 = ReadUint(in);

    int Offset = 0;
    int v9 = 0;
    //texels
    if (count2)
    {
        Graphics = (Graphic *)malloc (count2 * sizeof(Graphic));
        
        do
        {
          Graphics[v9].Mode = ReadByte(in);
          Graphics[v9].ClipW = ReadByte(in);
          Graphics[v9].ClipH = ReadByte(in);
          Graphics[v9].Width = ReadByte(in);
          Graphics[v9].Height = ReadByte(in);
          Graphics[v9].Count = ReadUint(in);
          Graphics[v9].Size = ReadWord(in);
          Graphics[v9].Type = 0;
          
          if(Graphics[v9].Width == 0)
            Graphics[v9].Width = 256;
          
          if ( Graphics[v9].Mode & 0x80 )
          {
            Graphics[v9].Type |= 0x40u;
            Graphics[v9].Mode &= 0xFu;
          }

          Graphics[v9].Offset = Offset;
          Graphics[v9].OutSize = (Graphics[v9].Count - Offset);
          Offset = Graphics[v9].Count;
          ++v9;
        }
        while ( v9 < count2 );
    }

    //palettes
    int v20 = 0;
    int v21 = 0;
    Offset = 0;
    if ( count1 )
    {
        Palettes = (Palette *)malloc (count1 * sizeof(Palette));
        do
        {
          int v23 = ReadUint(in);
          Palettes[v21].Size = (v23 - v20) >> 1;
          Palettes[v21].Count = v23;
          v20 = v23;
          Palettes[v21].Offset = Offset;
          Palettes[v21].OutSize = (Palettes[v21].Count - Offset);
          Offset = Palettes[v21].Count;
          
          ++v21;
        }
        while ( v21 < count1 );
    }
    
    int v25 = 0;
    if (count3)//models
    {
        do
        {
          int v2480 = 0;
          int v2488 = 0;
          int v27 = ReadUint(in);
          int v35252 = v27;
          int v35260 = v27;
          int v30 = v35260;

          ++v25;
        }
        while ( v25 < count3);
    }

    int v32 = 0;
    if (count4)
    {
        do
        {
          int vv1 = ReadByte(in);
          int vv2 = ReadWord(in);
          int vv3 = ReadWord(in);
          int vv4 = ReadWord(in);
          int vv5 = ReadByte(in);
          
          for(int i = 0; i < vv5; i++)
            ReadByte(in);
          
          PrintfPorcentaje(v32,count4-1,true, 10,"Extrayendo Texturas Y Sprites......");
          ExtraerTexels(vv4, vv3);
          
          v32++;
        }
        while ( v32 < count4);
    }
    fclose(in);
    
    LoadHeaders();
    ExtraerSFX();

    CloseZip(hz);
    system("PAUSE");
    return EXIT_SUCCESS;
}
