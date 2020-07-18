#ifndef WFT2_HPP
#define WFT2_HPP

#include <iostream>
#include <vector>

extern "C" typedef struct FT_FaceRec_* FT_Face;
extern "C" typedef struct FT_LibraryRec_* FT_Library;

namespace wft {


struct mask_bits
{
  mask_bits(void) {};
  mask_bits(const FT_Face& F);

  std::vector<unsigned char> Bits {};
  uint32_t symbol_uncode = 0;
  int top = 0;
  int left = 0;
  int width  = 0;
  int height = 0;
};


///
/// \brief The wft_lib class
///
class lib
{
  private:
    lib(const lib &) = delete;
    lib &operator =(const lib &) = delete;

    FT_Library FtLib {};

  public:
    lib(void);
    ~lib(void);
    operator FT_Library() const { return FtLib; }

};


///
/// \brief The wft_face class
///
class font
{
  private:
    font(const font &) = delete;
    font &operator =(const font &) = delete;

    lib wFtLib {};
    FT_Face FtFace {};

    int get_kerning(unsigned int char_first, unsigned int char_second, unsigned int kern_mode = 0);
    void get_bbox(mask_bits& Image);

  public:
    font(const char *filename, int face_index = 0);
    ~font(void);
    operator FT_Face() const { return FtFace; }

    void set_size(unsigned int w, unsigned int h);
    mask_bits get_symbol(uint32_t symbol_code);
    mask_bits make_bitmap_text(const std::string& Text);
};


} // namespace tr

#endif // WFT2_HPP
