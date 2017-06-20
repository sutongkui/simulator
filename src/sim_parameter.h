
//spring parameters used both in host and device

const int NUM_PER_VERTEX_ADJ_FACES = 20;        //与simulation中的NUM_PER_VERTEX_ADJ_FACES一致
const unsigned int NUM_PER_VERTEX_SPRING_STRUCT = 20;    //一级邻域的最大数目，若小于NUM_PER_VERTEX_SPRING_STRUCT，以MAX_INT结尾
const unsigned int NUM_PER_VERTEX_SPRING_BEND = 20;    //二级邻域的最大数目，若小于NUM_PER_VERTEX_SPRING_BEND，以MAX_INT结尾
const unsigned int SENTINEL = UINT_MAX;     //每个点最大包含NUM_PER_VERTEX_ADJ_FACES个邻近面，不足者以SENTINEL作为结束标志



