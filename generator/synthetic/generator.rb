require 'json'
require 'logger'

class InstanceGenerationError < StandardError
end

class InstanceGenerator
  def initialize(params)
    @m = params[:m].to_i # Number of trays
    @delta = params[:delta].to_i # Number of tasks per tray
    @omega = params[:omega].to_i # Time horizon size
    @greedy_prob = params[:greedy_prob].to_f # Probability of greedily assigning a task to a tray
    @tw_mult = params[:tw_multiplier].to_f # Time window size multiplier

    @tw_size = (@tw_mult * @omega / (@m * @delta)).ceil # Half-tw size
    @rng = Random.new
    @log = Logger.new(STDOUT)
    @log.level = Logger::WARN
    
    raise InstanceGenerationError.new "Delta must be at least 2 (start and end task)" if @delta < 2
    
    generate_tasks
    assign_tasks_to_trays
    assign_trays_to_shelves
    generate_tws
  end
  
  def generate_tasks
    n_tasks = @m * @delta
    max_len = [1, (@omega / n_tasks).floor].max
    
    task_start_times = 1.upto(@omega - @tw_size - 1).to_a.shuffle.take(n_tasks).sort
        
    task_durations = task_start_times.map.with_index do |time, idx|
      time_to_next = (idx < task_start_times.size - 1) ? task_start_times[idx+1] - time : 1
      r = @rng.rand(1..max_len)
      [r, time_to_next].min
    end
        
    @backbone = task_start_times.zip(task_durations).map{|st,du| {start_time: st, duration: du}}
  end
  
  def assign_tasks_to_trays
    @tasks = Array.new(@m) {Array.new}

    # Assign start tasks
    0.upto(@m - 1) do |t|
      @log.debug "Tray #{t+1}..."
      
      start_task = @backbone.select{|b| b[:start_time] < 0.8 * @omega}.sample
      @log.debug "\tStart task at time #{start_task[:start_time]}"
      
      @tasks[t] << start_task
      @log.debug "\tCurrent task list: #{@tasks[t].inspect}"
      
      @backbone.delete(start_task)
      @log.debug "\tCurrent number of tasks left in the backbone: #{@backbone.size}"
    end
    
    if @backbone.size != @m * (@delta - 1)
      raise InstanceGenerationError.new "Number of tasks left in the backbone is not correct: #{@backbone.size} vs. #{@m * (@delta-1)}"
    end

    # Assign other tasks
    0.upto(@m - 1) do |t|
      @log.debug "Tray #{t+1}..."
      task_idx = 0
      
      while task_idx < @backbone.size and @tasks[t].size < @delta
        @log.debug "\tConsidering task at index #{task_idx} (#{@backbone[task_idx].inspect})"
        if @rng.rand < @greedy_prob
          @log.debug "\tTaking that task!"
          
          @tasks[t] << @backbone.delete_at(task_idx)
          @log.debug "\tNew task list for #{t+1}: #{@tasks[t].inspect}"
          @log.debug "\tCurrent number of tasks left in the backbone: #{@backbone.size}"
        else
          @log.debug "\tNot taking that task!"
          task_idx += 1
        end
      end
    end
    
    # If it was not possible to find enough tasks...
    0.upto(@m - 1) do |t|
      if @tasks[t].size < @delta
        @log.debug "Tray #{t+1} only has #{@tasks[t].size} tasks instead of #{@delta}"
        @log.debug "\tTasks: #{@tasks[t].inspect}"
        @log.debug "\tCurrent start times in the backbone: #{@backbone.map{|x| x[:start_time]}.inspect}"
        
        needed = @delta - @tasks[t].size
        latest_task = @tasks[t].max{|t1, t2| t1[:start_time] <=> t2[:start_time]}
        later_tasks = @backbone.select{|b| b[:start_time] > latest_task[:start_time] + latest_task[:duration]}
        
        if later_tasks.empty?
          @log.debug "\tNo task is eligible to complete the task list of #{t+1}: it will have fewer tasks!"
          next
        end
        
        @log.debug "\tTaking #{needed} tasks starting after #{latest_task[:start_time]} + #{latest_task[:duration]}"
        @log.debug "\tEligible start times: #{later_tasks.map{|x| x[:start_time]}.inspect}"
        
        @tasks[t] += later_tasks[0..(needed - 1)]
        later_tasks[0..(needed - 1)].each{|b| @backbone.delete(b)}
        @log.debug "\tNew list of tasks for #{t+1}: #{@tasks[t].inspect}"
        @log.debug "\tCurrent number of tasks left in the backbone: #{@backbone.size}"
      end
      
      if @tasks[t].size < 2
        raise InstanceGenerationError.new "Could not assign even 2 tasks to tray #{t}"
      end
    end
    
    @tasks.each do |tray_tasks|
      tray_tasks.sort! {|t1,t2| t1[:start_time] <=> t2[:start_time]}
    end
  end
  
  def assign_trays_to_shelves
    free = [1.upto(@omega).to_a]
    @tray_shelf = Array.new(@m) {0}
    
    0.upto(@m - 1) do |t|
      start_t = @tasks[t].map{|x| x[:start_time]}.min
      end_t = @tasks[t].map{|x| x[:start_time] + x[:duration]}.max
      life = (start_t..end_t).to_a
      
      0.upto(free.size - 1) do |s|
        if (free[s] & life).size == life.size # Shelf is always free during the tray's life
          @tray_shelf[t] = s + 1
          free[s] -= life
          break
        end
      end
      
      if @tray_shelf[t] == 0 # No available shelf found: create new
        @tray_shelf[t] = free.size + 1
        free << 1.upto(@omega).to_a - life
      end
    end
  end
  
  def generate_tws
    @tasks.each do |tray_tasks|
      min_start_time, max_start_time = tray_tasks.map{|x| x[:start_time]}.minmax
      start_task_duration = tray_tasks.filter{|x| x[:start_time] == min_start_time}.first[:duration]
      tray_tasks.each do |task|
        if task[:start_time] == min_start_time # Planting task!
          task[:tw_start] = task[:start_time] - @rng.rand(0..@tw_size)
          task[:tw_start] = [1, task[:tw_start]].max
          task[:tw_end] = task[:start_time] + @rng.rand(0..@tw_size)
          task[:tw_end] = [@omega, task[:tw_end]].min
          task[:type] = 'planting'
        else
          task[:tw_start] = (task[:start_time] - min_start_time) - @rng.rand(0..@tw_size)
          task[:tw_start] = [start_task_duration, task[:tw_start]].max
          task[:tw_end] = (task[:start_time] - min_start_time) + @rng.rand(0..@tw_size)
          task[:tw_end] = [task[:tw_start], task[:tw_end]].max
          task[:tw_end] = [@omega, task[:tw_end]].min
          task[:type] = (task[:start_time] == max_start_time) ? 'harvest' : 'other'
        end
      end
    end
  end
  
  def get_json
    return {
      n_shelves: @tray_shelf.max,
      n_trays: @m,
      time_horizon_len: @omega,
      trays: 0.upto(@m - 1).map do |t|
        {
          shelf: @tray_shelf[t],
          tasks: @tasks[t].map do |task|
            {
              start: task[:tw_start],
              end: task[:tw_end],
              duration: task[:duration],
              type: task[:type]
            }
          end
        }
      end
    }
  end
end

[150, 200, 250].each do |omega|
  [4, 5, 6].each do |delta|
    [14, 16, 18].each do |m|
      [0.2, 0.4, 0.6].each do |greedy_prob|
        [1.0, 1.5, 2.0].each do |tw_multiplier|
          fn = "../../data/synthetic/inst-#{omega}-#{delta-2}-#{m}-#{greedy_prob}-#{tw_multiplier}.json"
          next if File.exists?(fn)
        
          begin
            i = InstanceGenerator.new m: m, delta: delta, omega: omega, greedy_prob: greedy_prob, tw_multiplier: tw_multiplier 
            File.write(fn, JSON.pretty_generate(i.get_json))
          rescue InstanceGenerationError => e
            puts "Cannot generate instance. Try again. (#{omega}, #{delta}, #{m}, #{greedy_prob}, #{tw_multiplier})"
            puts "\t=> #{e.message}"
          end
        end
      end
    end
  end
end