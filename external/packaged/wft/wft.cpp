#include <ft2build.h>
#include FT_FREETYPE_H
#include FT_OUTLINE_H

#include "wft.hpp"
#include <algorithm>

#ifdef _MSC_VER
#define __PRETTY_FUNCTION__ __FUNCSIG__
#endif

namespace wft {

///
/// \brief string2unicode
/// \param Text
/// \return Unicode vector
///
std::vector<uint32_t> string2unicode(const std::string& Text)
{
  std::vector<uint32_t> Result {};
  for(size_t i = 0; i < Text.size();)
  {
    int a=Text[i++];
    //if((a&0x80)==0);
    if((a&0xE0)==0xC0){
      a=(a&0x1F)<<6;
      a|=Text[i++]&0x3F;
    }else if((a&0xF0)==0xE0){
      a=(a&0xF)<<12;
      a|=(Text[i++]&0x3F)<<6;
      a|=Text[i++]&0x3F;
    }else if((a&0xF8)==0xF0){
      a=(a&0x7)<<18;
      a|=(a&0x3F)<<12;
      a|=(Text[i++]&0x3F)<<6;
      a|=Text[i++]&0x3F;
    }
    Result.push_back(static_cast<unsigned int>(a));
  }
  return Result;
}


unsigned int unsig(int x)
{
  if(x < 0) return 0;
  else return static_cast<unsigned int>(x);
}


///
/// \brief wft::wft
///
lib::lib(void)
{
  FT_Error error = FT_Init_FreeType(&FtLib);
  if (error) std::cerr << "FT_Init_FreeType() failed" << std::endl;
}


///
/// \brief lib::~lib
///
lib::~lib(void)
{
  FT_Done_FreeType(FtLib);
}


///
/// \brief wft_face::wft_face
/// \param library
/// \param filename
///
font::font(const char *filename, int face_index)
{
  FT_Error error = FT_New_Face(wFtLib, filename, face_index, &FtFace);
  if (error) std::cerr << "FT_New_Face() failed" << std::endl;
}


///
/// \brief font::~font
///
font::~font(void)
{
  FT_Done_Face(FtFace);
}


///
/// \brief wft_face::set_pixel_size
/// \param w
/// \param h
///
void font::set_size(unsigned int w, unsigned int h)
{
  FT_Error error = FT_Set_Pixel_Sizes(FtFace, w, h);
  if(error) std::cerr << "FT_Set_Pixel_Sizes() failed" << std::endl;
}


///
/// \brief wft_face::get_kerning
/// \param leftCharcode
/// \param rightCharcode
/// \return Кернинг двух символов
///
int font::get_kerning(unsigned int char_first, unsigned int char_second, unsigned int kern_mode)
{
  if(kern_mode == 0) kern_mode = FT_KERNING_DEFAULT;

  if(char_first == 0) return 0;
  FT_UInt IndexFirst = FT_Get_Char_Index(FtFace, char_first);
  FT_UInt IndexSecond = FT_Get_Char_Index(FtFace, char_second);
  FT_Vector delta {};
  FT_Error error = FT_Get_Kerning(FtFace, IndexFirst, IndexSecond, kern_mode, &delta);
  if( error ) std::cerr << std::endl << __PRETTY_FUNCTION__ << std::endl
                        << "FT_Get_Kerning(FtFace, IndexFirst, IndexSecond, FT_KERNING_DEFAULT, &delta) failed"
                        << std::endl;
  return delta.x >> 6; // x  в формате 26.6, поэтому сдвигаем результат
}


///
/// \brief image::image
/// \param Bitmap
///
mask_bits::mask_bits(const FT_Face& F)
{
  width  = static_cast<int>(F->glyph->bitmap.width);
  height = static_cast<int>(F->glyph->bitmap.rows);
  top = F->glyph->bitmap_top;
  left = F->glyph->bitmap_left;

  Bits.resize(unsig(width) * unsig(height), 0xFF);
  memcpy(Bits.data(), F->glyph->bitmap.buffer, Bits.size());
}


///
/// \brief paint_over
/// \param Src
/// \param Dst
/// \param x
/// \param y
///
void paint_over(const mask_bits& Src, mask_bits& Dst, int x, int y)
{
  if((Src.width + x  > Dst.width ) || (Src.height + y > Dst.height ))
  {
    std::cerr << std::endl << __PRETTY_FUNCTION__ << std::endl
                            << "ERROR: (Src.width + x  > Dst.width ) || (Src.height + y > Dst.height )" << std::endl;
    return;
  }

  int i = 0;                             // число скопированных пикселей
  int i_max = Src.height * Src.width;    // сумма пикселей источника, которые надо скопировать
  int src_row_start = 0;                 // индекс в начале строки источника
  int dst_row_start = x + y * Dst.width; // индекс начального пикселя приемника

  while(i < i_max)
  {
    int row_n = Src.width;
    unsigned int dst = unsig(dst_row_start);
    unsigned int src = unsig(src_row_start);
    while(row_n > 0)
    {
      Dst.Bits[dst] = Src.Bits[src];
      dst += 1;
      src += 1;
      row_n -= 1;
      i += 1;
    }
    src_row_start += Src.width; // переход на начало следующей строки источника
    dst_row_start += Dst.width; // переход на начало следующей строки приемника
  }
}


///
/// \brief wft_face::get_symbol
/// \param symbol_code
/// \return
///
mask_bits font::get_symbol(uint32_t symbol_code)
{
  FT_Error error = FT_Load_Char(FtFace, symbol_code, FT_LOAD_RENDER);

  if( error )
  {
    std::cerr << std::endl << __PRETTY_FUNCTION__ << std::endl
                        << "FT_Load_Char(FtFace, symbol_code, FT_LOAD_RENDER) failed" << std::endl;
    return mask_bits {};
  }
  mask_bits Result { FtFace };
  Result.symbol_uncode = symbol_code;

  return Result;
}


///
/// \brief wft_face::get_symbols_row
/// \param TextUnicode
/// \return
///
mask_bits font::make_bitmap_text(const std::string& Text)
{
  std::vector<uint32_t> TextUnicode = string2unicode(Text);
  mask_bits Result {};
  std::vector<mask_bits> TmpArray {};

  for(auto s: TextUnicode)
  {
    TmpArray.push_back(get_symbol(s));


    Result.height = std::max(Result.height, TmpArray.back().height);
    Result.width += TmpArray.back().width + TmpArray.back().left;

    auto ts = TmpArray.size();
    if(ts > 1)
    {
      Result.width += get_kerning(TmpArray[ts - 2].symbol_uncode, TmpArray[ts - 1].symbol_uncode);

      // make "Space"
      if(TmpArray[ts - 2].width == 0)
      {
        TmpArray.back().left += static_cast<int>(TmpArray.back().width);
        Result.width += TmpArray.back().left;
      }
    }

    // Здесь назначение .top меняется - тут это максимальное смещение нижней границы
    Result.top = std::max(Result.top, static_cast<int>(TmpArray.back().height) - TmpArray.back().top);
  }

  Result.height += Result.top;

  Result.Bits.resize(unsig(Result.width) * unsig(Result.height), 0x00);

  int x = 0;
  uint32_t prev_symbol_unicode = 0;
  for(auto& M: TmpArray)
  {
    x += M.left + get_kerning(prev_symbol_unicode, M.symbol_uncode);
    paint_over(M, Result, x, Result.height - M.top - Result.top);
    x += static_cast<int>(M.width);
  }

  return Result;
}

} // namespace tr
