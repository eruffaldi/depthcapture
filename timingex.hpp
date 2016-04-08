/**
 * Simple class with statistics for service time and period 
 */
#pragma once
#include <iostream>
#include <chrono>

template <class T>
 struct UnitName
 {
 };

template <>
 struct UnitName<std::chrono::microseconds>
 {
 	static constexpr const char *  name = "us";
 };

template <>
 struct UnitName<std::chrono::milliseconds>
 {
 	static constexpr const char * name = "ms";
 };

template <>
 struct UnitName<std::chrono::seconds>
 {
 	static constexpr const char * name = "s";
 };

	struct Stat
	{
		double max_=0;
		double min_=0;
		double mean_=0;
		double m2_=0;
		int n_=0;
		double first_=0;

		double mean() const { return mean_; }
		double std() const { return n_ < 2 ? 0 : m2_/(n_-1); }
		double min() const { return min_; }
		double max() const { return max_; }
		int count() const { return n_; }

		void reset()
		{
			*this = Stat();
		}

		void append(double e)
		{
			if(n_ == 0)
			{
				first_ = e;
				max_ = min_ = mean_ = e;
				m2_ = 0;
				n_ = 1;
			}
			else
			{
				max_ =  max_ < e ? e: max_;
				min_ =  min_ > e ? e: min_;
				n_++;
				auto d = e-mean_;
				mean_ += d/n_;
				m2_ += (e-mean_)*d;
			}
		}
	};

template <class T = std::chrono::steady_clock, class UT=std::chrono::milliseconds>
struct PeriodTiming
{
public:	


	using base_t = T;
	using time_t = typename T::time_point;

	PeriodTiming(std::string timer_name = "")	: name(timer_name)
	{
		start_time = base_t::now();
	}

	void reset()
	{
		*this = PeriodTiming(name);
	}
	
	void start()
	{
		auto now = base_t::now();
		if (period.count() > 0)
		{
			auto time = std::chrono::duration_cast<UT>(now - start_time).count();
			service_time.append(time);
		}
		start_time = now;
	}

	void stop()
	{
		double time = std::chrono::duration_cast<UT>(base_t::now() - start_time).count();
		period.append(time);
	}

	void statreset()
	{
		period.reset();
		service_time.reset();
	}

//	double total_elapsed() const {		
//	}

	int count() const { return period.count(); }
    
	std::string name;
	time_t start_time;
	// TODO really first time_t start_time;
	Stat period;
	Stat service_time;
};

std::ostream & operator << (std::ostream & os, const  Stat & t)
{
	os << " n=" << t.count() <<  " mu=" << t.mean() << " stat=" << t.std() << " max=" << t.max() << " min=" << t.min();
	return os;
}


template <class T,class U>
std::ostream & operator << (std::ostream & os, const PeriodTiming<T,U> & t)
{
	os << "PT[" << t.name << " in " << UnitName<U>::name << " period[" << t.period << "] servicetime[" << t.service_time << "]";
	return os;
}

