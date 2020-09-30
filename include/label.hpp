/** @file
 *  @brief 机器人在线运行框架
 *  @author 鞠孝亮 (JU Xiaoliang)
 *  @date 2017.11
 *  @note 本项目使用MIT开源协议，请遵守该协议使用
 */


#ifndef LABEL_HPP
#define LABEL_HPP

/**
 * @brief 实现数据标签管理类
 */

class Label {
public:
    enum LabelType {
        UNKNOWN = 0,
        DONTCARE = 1 << 0,
//        EDGEPT = 1 << 1,
        TRAVESABLE = 1 << 2,
        NONTRAVESABLE = 1 << 3,
        POSSIOBSTA = 1 << 4,
        NEGATOBSTA = 1 << 5,
        HANGDOWNTR = 1 << 6,
        HANGDOWNUN = 1 << 7,
        FLATGROUND = 1 << 8,
        DOWNSLOPE = 1 << 9,
        UPSLOPE = 1 << 10,
        LEFTSIDESLOPE = 1 << 11,
        RIGHTSIDESLOPE = 1 << 12,
        EDGEPOINTS = 1 << 13,
//        NONVALID = 1 << 14
    };/**< 使用二进制位定义可能需要的各个类别 */


    Label(int type = 0) : type(type) {}
    
    /**
     * @brief 添加B所具备的所有标签
     * @param B 标签对象B
     */
    bool set(Label B) {
        /*
         * 使用位或运算符实现标签设置
         */
        type = B.type | type;
        return true;
    }
    
    /**
     * @brief 添加标签值为B
     * @param B 标签值B
     */
    bool set(LabelType B){
        /*
         * 使用位或运算符实现标签设置
         */
        type = B | type;
        return true;
    }
    
    /**
     * @brief 判断该标签类是否含有标签值B
     * @param B 标签值B
     * @return bool 含有B标签返回true
     */
    bool is(LabelType A) const{
        /*
         * 使用标签存储所在的int整数判断两包含关系
         */
        if (A == LabelType::UNKNOWN) {
            return type == 0;
        } else {
            return static_cast<int>(type & A) == static_cast<int>(A);
        }
    }
    
    /**
     * @brief 判断该标签类的所有标签是含有标签类B的所有标签
     * @param B 标签对象B
     * @return bool 包含B返回true
     */
    bool is(Label A) const{
        /*
         * 使用标签存储所在的int整数判断两包含关系
         */
        if (A.type == LabelType::UNKNOWN) {
            return type == 0;
        } else {
            return static_cast<int>(type & A.type) == static_cast<int>(A.type);
        }
    }
    
    /**
     * @brief 清空标签类含有的所有标签值
     */
    void reset(){
        /*
         * 重置为UNKOWN标签
         */
        type = 0;
    }
    
    /**
     * @brief 删除标签类含有的B的所有标签值
     * @param B 标签对象B
     */
    void erase(Label B){
        /*
         * 使用位与运算符实现标签设置
         */
        type &= ~B.type;
    }


    /**< 按位取反运算符 */
    Label operator ~(){
        Label ans(~type);
        return ans;
    }
    
    /**< 位与运算符 */
    Label operator &(const Label &other){
        type &= other.type;
        return *this;
    }

    int type;/**< 标签实际存储位置 */

};

#endif // LABEL_HPP
