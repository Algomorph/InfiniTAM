#ifndef WFT2_HPP
#define WFT2_HPP

#include <iostream>
#include <vector>

extern "C" typedef struct FT_FaceRec_* FT_Face;
extern "C" typedef struct FT_LibraryRec_* FT_Library;

namespace wft {


struct glyph_bitmap
{
  glyph_bitmap(void) {};
  glyph_bitmap(const FT_Face& face, uint32_t symbol_unicode);

  std::vector<unsigned char> data {};
  uint32_t code_unicode = 0;
  int top = 0;
  int left = 0;
  int advance_x = 0;
  int advance_y = 0;
  int lsb_delta = 0;
  int rsb_delta = 0;
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
class face
{
  private:
    face(const face &) = delete;
    face &operator =(const face &) = delete;

    lib wFtLib {};
    FT_Face FtFace {};

    int get_kerning(unsigned int char_first, unsigned int char_second, unsigned int kern_mode = 0);
    void get_bbox(glyph_bitmap& Image);

  public:
    face(const char *filename, int face_index = 0);
    ~face(void);
    operator FT_Face() const { return FtFace; }

    void set_size(unsigned int w, unsigned int h);
    glyph_bitmap get_symbol(uint32_t symbol_code);
    glyph_bitmap make_bitmap_text(const std::string& Text);
};


} // namespace tr

#endif // WFT2_HPP
