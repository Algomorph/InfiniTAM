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
/// \param text
/// \return Unicode vector
///
std::vector<uint32_t> string2unicode(const std::string& text) {
	std::vector<uint32_t> unicode{};
	for (size_t i = 0; i < text.size();) {
		int a = text[i++];
		//if((a&0x80)==0);
		if ((a & 0xE0) == 0xC0) {
			a = (a & 0x1F) << 6;
			a |= text[i++] & 0x3F;
		} else if ((a & 0xF0) == 0xE0) {
			a = (a & 0xF) << 12;
			a |= (text[i++] & 0x3F) << 6;
			a |= text[i++] & 0x3F;
		} else if ((a & 0xF8) == 0xF0) {
			a = (a & 0x7) << 18;
			a |= (a & 0x3F) << 12;
			a |= (text[i++] & 0x3F) << 6;
			a |= text[i++] & 0x3F;
		}
		unicode.push_back(static_cast<unsigned int>(a));
	}
	return unicode;
}

const char* get_freetype_error_message(FT_Error err) {
#undef FTERRORS_H_
#define FT_ERRORDEF(e, v, s)  case e: return s;
#define FT_ERROR_START_LIST     switch (err) {
#define FT_ERROR_END_LIST       }

#include FT_ERRORS_H
	return "(Unknown error)";
}

unsigned int unsig(int x) {
	if (x < 0) return 0;
	else return static_cast<unsigned int>(x);
}

/**
 * \brief Initialize the Freetype library.
 */
lib::lib(void) {
	FT_Error error = FT_Init_FreeType(&FtLib);
	if (error) std::cerr << "FT_Init_FreeType() failed" << std::endl;
}

lib::~lib(void) {
	FT_Done_FreeType(FtLib);
}


///
/// \brief wft_face::wft_face
/// \param library
/// \param filename
///
face::face(const char* filename, int face_index) {
	FT_Error error = FT_New_Face(wFtLib, filename, face_index, &FtFace);
	if (error) std::cerr << "FT_New_Face() failed" << std::endl;
}


///
/// \brief font::~font
///
face::~face(void) {
	FT_Done_Face(FtFace);
}


///
/// \brief wft_face::set_pixel_size
/// \param w
/// \param h
///
void face::set_size(unsigned int w, unsigned int h) {
	FT_Error error = FT_Set_Pixel_Sizes(FtFace, w, h);
	if (error) std::cerr << "FT_Set_Pixel_Sizes() failed" << std::endl;
}


///
/// \brief wft_face::get_kerning
/// \param leftCharcode
/// \param rightCharcode
/// \return Кернинг двух символов
///
int face::get_kerning(unsigned int char_first, unsigned int char_second, unsigned int kern_mode) {
	if (kern_mode == 0) kern_mode = FT_KERNING_DEFAULT;

	if (char_first == 0) return 0;
	FT_UInt IndexFirst = FT_Get_Char_Index(FtFace, char_first);
	FT_UInt IndexSecond = FT_Get_Char_Index(FtFace, char_second);
	FT_Vector delta{};
	FT_Error error = FT_Get_Kerning(FtFace, IndexFirst, IndexSecond, kern_mode, &delta);
	if (error)
		std::cerr << std::endl << __PRETTY_FUNCTION__ << std::endl
		          << "FT_Get_Kerning(FtFace, IndexFirst, IndexSecond, FT_KERNING_DEFAULT, &delta) failed"
		          << std::endl;
	return delta.x >> 6; // x  в формате 26.6, поэтому сдвигаем результат
}


/**
 * \brief a bitmap containing one or more glyphs
 * \param face 
 */
glyph_bitmap::glyph_bitmap(const FT_Face& face, uint32_t code) :
		code_unicode(code),
		width(static_cast<int>(face->glyph->bitmap.width)),
		height(static_cast<int>(face->glyph->bitmap.rows)),
		top(face->glyph->bitmap_top),
		left(face->glyph->bitmap_left),
		advance_x(face->glyph->advance.x),
		advance_y(face->glyph->advance.y),
		lsb_delta(face->glyph->lsb_delta),
		rsb_delta(face->glyph->rsb_delta) {
	data.resize(unsig(width) * unsig(height), 0xFF);
	memcpy(data.data(), face->glyph->bitmap.buffer, data.size());
}


/**
 * \brief paint one glyph bitmap over the other at the specified location
 * \details assumes the destination is big enough to accommodate the entire size of the source at the given location.
 * \param source source bitmap
 * \param destination destination bitmap
 * \param x horizontal offset of source bitmap origin from destination bitmap origin
 * \param y vertical offset of source bitmap origin from destination bitmap origin
 */
void paint_over(const glyph_bitmap& source, glyph_bitmap& destination, int x, int y) {
	if ((source.width + x > destination.width) || (source.height + y > destination.height)) {
		std::cerr << std::endl << __PRETTY_FUNCTION__ << std::endl
		          << "ERROR: (source.width + x > destination.width) || (source.height + y > destination.height)" << std::endl;
		return;
	}
	int destination_row_start = x + y * destination.width;
	int i_source_pixel = 0;
	for (int y_source = 0; y_source < source.height; y_source++) {
		unsigned int i_destination_pixel = unsig(destination_row_start);
		for (int x_source = 0; x_source < source.width; x_source++, i_destination_pixel++, i_source_pixel++) {
			destination.data[i_destination_pixel] = source.data[i_source_pixel];
		}
		destination_row_start += destination.width;
	}
}


///
/// \brief wft_face::get_symbol
/// \param symbol_code
/// \return
///
glyph_bitmap face::get_symbol(uint32_t symbol_code) {
	FT_Error error = FT_Load_Char(FtFace, symbol_code, FT_LOAD_RENDER);
	if (error) {
		const char* error_message = get_freetype_error_message(error);
		std::cerr << std::endl << __PRETTY_FUNCTION__ << std::endl
		          << "FT_Load_Char(FtFace, symbol_code, FT_LOAD_RENDER) failed with message '" << error_message << "'" << std::endl;

		return glyph_bitmap{};
	}
	glyph_bitmap symbol(FtFace, symbol_code);

	return symbol;
}


///
/// \brief wft_face::get_symbols_row
/// \param TextUnicode
/// \return
///
glyph_bitmap face::make_bitmap_text(const std::string& Text) {
	std::vector<uint32_t> text_unicode = string2unicode(Text);
	glyph_bitmap bitmap{};
	std::vector<glyph_bitmap> individual_glyphs{};

	for (auto symbol_code: text_unicode) {
		individual_glyphs.push_back(get_symbol(symbol_code));

		int pixel_width = individual_glyphs.back().advance_x >> 6; // 26.6 pixel float format --> pixels

		bitmap.height = std::max(bitmap.height, individual_glyphs.back().height);
		bitmap.width += pixel_width;

		auto glyph_count = individual_glyphs.size();
		if (glyph_count > 1) {
			bitmap.width += get_kerning(individual_glyphs[glyph_count - 2].code_unicode, individual_glyphs[glyph_count - 1].code_unicode);
		}

		// Здесь назначение .top меняется - тут это максимальное смещение нижней границы
		bitmap.top = std::max(bitmap.top, static_cast<int>(individual_glyphs.back().height) - individual_glyphs.back().top);
	}

	bitmap.height += bitmap.top;

	bitmap.data.resize(unsig(bitmap.width) * unsig(bitmap.height), 0x00);

	int x = 0;
	uint32_t prev_symbol_unicode = 0;
	for (auto& glyph: individual_glyphs) {
		paint_over(glyph, bitmap, x + glyph.left, bitmap.height-bitmap.top-glyph.top);
		// 26.6 format for advancing the character
		x += (glyph.advance_x >> 6);
	}

	return bitmap;
}

} // namespace tr
