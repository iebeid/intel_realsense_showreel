#define POW2(a) ((a) * (a))

__kernel void filter(               
   __global uchar* input,                   
   __global uchar* output)                  
{                                           
   int i = get_global_id(0);                
   output[i] = 255 - input[i];              
};


__constant sampler_t samplerLN = CLK_NORMALIZED_COORDS_FALSE | CLK_ADDRESS_CLAMP_TO_EDGE | CLK_FILTER_LINEAR;

__kernel void shift(const image2d_t src, float shift_x, float shift_y, __global uchar* dst, int dst_step, int dst_offset, int dst_rows, int dst_cols)
{
	int x = get_global_id(0);
	int y = get_global_id(1);
	if (x >= dst_cols) return;
	int dst_index = mad24(y, dst_step, mad24(x, (int)sizeof(dstT), dst_offset));
	__global dstT *dstf = (__global dstT *)(dst + dst_index);
	float2 coord = (float2)((float)x + 0.5f + shift_x, (float)y + 0.5f + shift_y);
	dstf[0] = (dstT)read_imagef(src, samplerLN, coord).x;
}


__kernel void bilateral_filter(global float * in, global float * out, const  float radius, const  float preserve)
{
	int gidx = get_global_id(0);
	int gidy = get_global_id(1);
	int n_radius = ceil(radius);
	int dst_width = get_global_size(0);
	int src_width = dst_width + n_radius * 2;

	int u, v, i, j;
	float4 center_pix =
		in[(gidy + n_radius) * src_width + gidx + n_radius];
	float4 accumulated = 0.0f;
	float4 tempf = 0.0f;
	float  count = 0.0f;
	float  diff_map, gaussian_weight, weight;

	for (v = -n_radius; v <= n_radius; ++v)
	{
		for (u = -n_radius; u <= n_radius; ++u)
		{
			i = gidx + n_radius + u;
			j = gidy + n_radius + v;

			int gid1d = i + j * src_width;
			tempf = in[gid1d];

			diff_map = exp(
				-(POW2(center_pix.x - tempf.x)
				+ POW2(center_pix.y - tempf.y)
				+ POW2(center_pix.z - tempf.z))
				* preserve);

			gaussian_weight =
				exp(-0.5f * (POW2(u) + POW2(v)) / radius);

			weight = diff_map * gaussian_weight;

			accumulated += tempf * weight;
			count += weight;
		}
	}
	out[gidx + gidy * dst_width] = accumulated / count;
}