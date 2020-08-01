# -*- coding:ascii -*-
from mako import runtime, filters, cache
UNDEFINED = runtime.UNDEFINED
STOP_RENDERING = runtime.STOP_RENDERING
__M_dict_builtin = dict
__M_locals_builtin = locals
_magic_number = 10
_modified_time = 1596308026.9804618
_enable_loop = True
_template_filename = '/home/algomorph/Workbench/InfiniTAM/Apps/Python/tuner/meshlab_transformation_filter_mako_template.mlx'
_template_uri = '/home/algomorph/Workbench/InfiniTAM/Apps/Python/tuner/meshlab_transformation_filter_mako_template.mlx'
_source_encoding = 'ascii'
_exports = []


def render_body(context,**pageargs):
    __M_caller = context.caller_stack._push_frame()
    try:
        __M_locals = __M_dict_builtin(pageargs=pageargs)
        val14 = context.get('val14', UNDEFINED)
        val10 = context.get('val10', UNDEFINED)
        val13 = context.get('val13', UNDEFINED)
        val1 = context.get('val1', UNDEFINED)
        val6 = context.get('val6', UNDEFINED)
        val5 = context.get('val5', UNDEFINED)
        val11 = context.get('val11', UNDEFINED)
        val15 = context.get('val15', UNDEFINED)
        val7 = context.get('val7', UNDEFINED)
        val8 = context.get('val8', UNDEFINED)
        val4 = context.get('val4', UNDEFINED)
        val3 = context.get('val3', UNDEFINED)
        val9 = context.get('val9', UNDEFINED)
        val0 = context.get('val0', UNDEFINED)
        val2 = context.get('val2', UNDEFINED)
        val12 = context.get('val12', UNDEFINED)
        __M_writer = context.writer()
        __M_writer('<!DOCTYPE FilterScript>\n<FilterScript>\n    <filter name="Matrix: Set/Copy Transformation">\n        <Param val5="')
        __M_writer(str(val5))
        __M_writer('" val0="')
        __M_writer(str(val0))
        __M_writer('" name="TransformMatrix" val13="')
        __M_writer(str(val13))
        __M_writer('" val15="')
        __M_writer(str(val15))
        __M_writer('" type="RichMatrix44f" val12="')
        __M_writer(str(val12))
        __M_writer('" val10="')
        __M_writer(str(val10))
        __M_writer('" val3="')
        __M_writer(str(val3))
        __M_writer('" val7="')
        __M_writer(str(val7))
        __M_writer('" val6="')
        __M_writer(str(val6))
        __M_writer('" val11="')
        __M_writer(str(val11))
        __M_writer('" val4="')
        __M_writer(str(val4))
        __M_writer('" val9="')
        __M_writer(str(val9))
        __M_writer('" val8="')
        __M_writer(str(val8))
        __M_writer('" val14="')
        __M_writer(str(val14))
        __M_writer('" val2="')
        __M_writer(str(val2))
        __M_writer('" tooltip="" val1="')
        __M_writer(str(val1))
        __M_writer('" description=""/>\n        <Param type="RichBool" tooltip="If selected, the new matrix will be composed with the current one (matrix=new*old)" value="false" description="Compose with current" name="compose"/>\n        <Param type="RichBool" tooltip="The transformation is explicitly applied, and the vertex coordinates are actually changed" value="true" description="Freeze Matrix" name="Freeze"/>\n        <Param type="RichBool" tooltip="If selected, the filter will be applied to all visible mesh layers" value="false" description="Apply to all visible Layers" name="allLayers"/>\n    </filter>\n</FilterScript>')
        return ''
    finally:
        context.caller_stack._pop_frame()


"""
__M_BEGIN_METADATA
{"filename": "/home/algomorph/Workbench/InfiniTAM/Apps/Python/tuner/meshlab_transformation_filter_mako_template.mlx", "uri": "/home/algomorph/Workbench/InfiniTAM/Apps/Python/tuner/meshlab_transformation_filter_mako_template.mlx", "source_encoding": "ascii", "line_map": {"16": 0, "37": 1, "38": 4, "39": 4, "40": 4, "41": 4, "42": 4, "43": 4, "44": 4, "45": 4, "46": 4, "47": 4, "48": 4, "49": 4, "50": 4, "51": 4, "52": 4, "53": 4, "54": 4, "55": 4, "56": 4, "57": 4, "58": 4, "59": 4, "60": 4, "61": 4, "62": 4, "63": 4, "64": 4, "65": 4, "66": 4, "67": 4, "68": 4, "69": 4, "75": 69}}
__M_END_METADATA
"""
